#!/usr/bin/env python3
"""
Test script for text_messaging module
Run this on a desktop computer to verify the module functions correctly
"""

import sys
sys.path.insert(0, 'lib')

from text_messaging import MessageQueue, APRSMessage, parse_serial_input

def test_message_queue():
    """Test MessageQueue functionality"""
    print("Testing MessageQueue...")
    
    queue = MessageQueue(max_size=3)
    
    # Add messages
    queue.add("CALL-1", "Hello")
    queue.add("CALL-2", "World")
    queue.add("CALL-3", "Test")
    
    assert queue.has_messages()
    
    # Add one more (should remove oldest)
    queue.add("CALL-4", "Fourth")
    
    # Get messages
    msg1 = queue.get_next()
    assert msg1['destination'] == "CALL-2"
    assert msg1['text'] == "World"
    
    msg2 = queue.get_next()
    assert msg2['destination'] == "CALL-3"
    
    msg3 = queue.get_next()
    assert msg3['destination'] == "CALL-4"
    
    assert not queue.has_messages()
    assert queue.get_next() is None
    
    print("✓ MessageQueue tests passed")


def test_aprs_message_format():
    """Test APRS message formatting"""
    print("Testing APRS message formatting...")
    
    # Test message without ID
    msg = APRSMessage.format_message("OE3ABC", "OE3XYZ", "Test message")
    assert "OE3ABC>APRFGT:OE3XYZ" in msg
    assert ":Test message" in msg
    
    # Test message with ID
    msg = APRSMessage.format_message("OE3ABC", "OE3XYZ", "Test", "123")
    assert "OE3ABC>APRFGT:OE3XYZ" in msg
    assert ":Test{123" in msg
    
    # Test broadcast message
    msg = APRSMessage.format_message("OE3ABC", "*", "Broadcast")
    assert "OE3ABC>APRFGT:*" in msg
    assert ":Broadcast" in msg
    
    # Test addressee is padded to 9 characters
    msg = APRSMessage.format_message("OE3ABC", "A", "Test")
    assert "A        :" in msg  # A padded to 9 chars
    
    # Test long addressee is truncated
    msg = APRSMessage.format_message("OE3ABC", "VERYLONGCALL-99", "Test")
    assert "VERYLONGC:" in msg  # Truncated to 9 chars
    
    print("✓ APRS message formatting tests passed")


def test_aprs_ack():
    """Test APRS acknowledgment formatting"""
    print("Testing APRS ACK...")
    
    ack = APRSMessage.format_ack("OE3ABC", "OE3XYZ", "123")
    assert "OE3ABC>APRFGT:OE3XYZ" in ack
    assert ":ack123" in ack
    
    print("✓ APRS ACK tests passed")


def test_aprs_parse():
    """Test APRS message parsing"""
    print("Testing APRS message parsing...")
    
    # Test regular message with ID
    packet = "OE3ABC>APRFGT:OE3XYZ   :Hello World{123"
    msg = APRSMessage.parse_message(packet)
    assert msg is not None
    assert msg['from'] == "OE3ABC"
    assert "OE3XYZ" in msg['to']
    assert msg['text'] == "Hello World"
    assert msg['msg_id'] == "123"
    assert not msg['is_ack']
    
    # Test message without ID
    packet = "OE3ABC>APRFGT:OE3XYZ   :Hello"
    msg = APRSMessage.parse_message(packet)
    assert msg is not None
    assert msg['text'] == "Hello"
    assert msg['msg_id'] is None
    
    # Test ACK
    packet = "OE3ABC>APRFGT:OE3XYZ   :ack123"
    msg = APRSMessage.parse_message(packet)
    assert msg is not None
    assert msg['is_ack']
    
    # Test broadcast
    packet = "OE3ABC>APRFGT:*        :Hello everyone"
    msg = APRSMessage.parse_message(packet)
    assert msg is not None
    assert "*" in msg['to']
    
    # Test invalid packet
    packet = "INVALID"
    msg = APRSMessage.parse_message(packet)
    assert msg is None
    
    # Test position packet (should return None)
    packet = "OE3ABC>APRFGT:!4807.04N/01131.00E>"
    msg = APRSMessage.parse_message(packet)
    assert msg is None
    
    print("✓ APRS message parsing tests passed")


def test_serial_input_parsing():
    """Test serial input parsing"""
    print("Testing serial input parsing...")
    
    # Valid message to specific station
    result = parse_serial_input("MSG:OE3XYZ:Hello")
    assert result == ("OE3XYZ", "Hello")
    
    # Valid broadcast
    result = parse_serial_input("MSG:*:Broadcast message")
    assert result == ("CQ", "Broadcast message")
    
    # Case insensitive
    result = parse_serial_input("msg:OE3ABC:Test")
    assert result == ("OE3ABC", "Test")
    
    # With extra whitespace
    result = parse_serial_input("  MSG:CALL-1:  Test message  ")
    assert result == ("CALL-1", "Test message")
    
    # Invalid format (no colon)
    result = parse_serial_input("MSG-INVALID")
    assert result is None
    
    # Invalid format (no text)
    result = parse_serial_input("MSG:CALL:")
    assert result is None
    
    # Not a message command
    result = parse_serial_input("HELP")
    assert result is None
    
    print("✓ Serial input parsing tests passed")


def run_all_tests():
    """Run all tests"""
    print("=" * 60)
    print("Running text_messaging module tests")
    print("=" * 60)
    
    try:
        test_message_queue()
        test_aprs_message_format()
        test_aprs_ack()
        test_aprs_parse()
        test_serial_input_parsing()
        
        print("=" * 60)
        print("✓ All tests passed!")
        print("=" * 60)
        return 0
        
    except AssertionError as e:
        print("=" * 60)
        print("✗ Test failed!")
        print(str(e))
        print("=" * 60)
        return 1


if __name__ == "__main__":
    sys.exit(run_all_tests())
