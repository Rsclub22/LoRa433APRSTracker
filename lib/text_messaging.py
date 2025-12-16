"""
Simple Text Messaging Module for LoRa APRS Tracker
Implements basic text message sending/receiving similar to MeshCom
"""

class MessageQueue:
    """Simple message queue for pending outgoing messages"""
    
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.messages = []
    
    def add(self, destination, text):
        """Add a message to the queue"""
        if len(self.messages) >= self.max_size:
            # Remove oldest message if queue is full
            self.messages.pop(0)
        
        self.messages.append({
            'destination': destination.upper().strip(),
            'text': text[:67]  # APRS message text limit
        })
    
    def get_next(self):
        """Get the next message from the queue"""
        if self.messages:
            return self.messages.pop(0)
        return None
    
    def has_messages(self):
        """Check if there are pending messages"""
        return len(self.messages) > 0
    
    def clear(self):
        """Clear all pending messages"""
        self.messages = []


class APRSMessage:
    """APRS message formatter and parser"""
    
    @staticmethod
    def format_message(from_call, to_call, text, msg_id=None):
        """
        Format an APRS message packet
        Format: :ADDRESSEE:message text{msg_id
        ADDRESSEE is 9 characters, padded with spaces
        """
        # Pad addressee to 9 characters
        addressee = to_call.upper().ljust(9)[:9]
        
        # Build message
        if msg_id is not None:
            msg = ":{}:{}{{{}".format(addressee, text[:67], msg_id)
        else:
            msg = ":{}:{}".format(addressee, text[:67])
        
        # Complete APRS packet
        packet = "{}>APRFGT{}".format(from_call.upper(), msg)
        
        return packet
    
    @staticmethod
    def format_ack(from_call, to_call, msg_id):
        """
        Format an APRS acknowledgment
        Format: :ADDRESSEE:ack{msg_id
        """
        addressee = to_call.upper().ljust(9)[:9]
        ack = ":{}:ack{}".format(addressee, msg_id)
        packet = "{}>APRFGT{}".format(from_call.upper(), ack)
        return packet
    
    @staticmethod
    def parse_message(packet):
        """
        Parse an incoming APRS message packet
        Returns dict with 'from', 'to', 'text', 'msg_id' or None if not a message
        """
        try:
            # Expected format: CALL>APRFGT:ADDRESSEE:text{msg_id
            if ':' not in packet:
                return None
            
            # Split sender and payload
            parts = packet.split('>', 1)
            if len(parts) != 2:
                return None
            
            from_call = parts[0].strip()
            
            # Get payload after destination
            payload_parts = parts[1].split(':', 1)
            if len(payload_parts) < 2:
                return None
            
            payload = payload_parts[1]
            
            # Check if it's a message (starts with :)
            if not payload.startswith(':'):
                return None
            
            # Parse addressee and message text
            msg_parts = payload[1:].split(':', 1)
            if len(msg_parts) < 2:
                return None
            
            to_call = msg_parts[0].strip()
            message_text = msg_parts[1]
            
            # Check for message ID
            msg_id = None
            if '{' in message_text:
                text_parts = message_text.rsplit('{', 1)
                message_text = text_parts[0]
                msg_id = text_parts[1] if len(text_parts) > 1 else None
            
            # Check for ack/rej
            is_ack = message_text.startswith('ack')
            is_rej = message_text.startswith('rej')
            
            return {
                'from': from_call,
                'to': to_call,
                'text': message_text,
                'msg_id': msg_id,
                'is_ack': is_ack,
                'is_rej': is_rej
            }
            
        except Exception:
            return None


def parse_serial_input(input_str):
    """
    Parse serial input for message commands
    Format: MSG:DESTINATION:message text
    or: MSG:*:message text (for broadcast)
    Returns (destination, text) or None
    """
    input_str = input_str.strip()
    
    if not input_str.upper().startswith('MSG:'):
        return None
    
    # Remove MSG: prefix
    msg_data = input_str[4:]
    
    # Split destination and text
    parts = msg_data.split(':', 1)
    if len(parts) < 2:
        return None
    
    destination = parts[0].strip()
    text = parts[1].strip()
    
    if not destination or not text:
        return None
    
    # Default to broadcast if empty
    if destination == '*' or destination == '':
        destination = 'CQ'
    
    return (destination, text)
