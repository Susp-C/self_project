import sys, rospy, signal, subprocess, time
from std_msgs.msg import Float64, Bool

# Global reference so signal handler can clean up
global_lines = None

def signal_handler(sig, frame):
    if global_lines:
        global_lines.release()
    sys.exit(0)

def shutdown():
    if global_lines:
        global_lines.release()
    rospy.logwarn("BATTERY VOLTAGE TOO LOW. COMMENCING SHUTDOWN PROCESS")
    time.sleep(5)
    subprocess.run(["sudo", "shutdown", "-h", "now"])

def main():
    global global_lines
    rospy.init_node("battery_monitor") 
    message_rate = 50
    rate = rospy.Rate(message_rate)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        import gpiod
    except ImportError:
        rospy.logerr("Failed to import gpiod. Please install it using: apt-get install python3-libgpiod")
        sys.exit(1)

    estop_pin_number = 5
    battery_pin1_number = 6
    battery_pin2_number = 13
    battery_pin3_number = 19

    try:
        chip = gpiod.Chip("gpiochip4")
    except Exception as e:
        rospy.logerr(f"Cannot open gpiochip4: {e}")
        sys.exit(1)

    # Note: the lines are returned in the order they were requested
    global_lines = chip.get_lines([estop_pin_number, battery_pin1_number, battery_pin2_number, battery_pin3_number])
    global_lines.request(consumer="battery_monitor", type=gpiod.LINE_REQ_DIR_IN)

    battery_percentage_publisher = rospy.Publisher("/battery_percentage", Float64, queue_size = 10)
    estop_publisher = rospy.Publisher("/emergency_stop_status", Bool, queue_size = 10)
    current_estop_bit = 0
    number_of_low_battery_detections = 0

    vals = global_lines.get_values()
    estop_bit = vals[0]
    battery_bit1 = vals[1]
    battery_bit2 = vals[2]
    battery_bit3 = vals[3]

    # Grab initial value and publish that immediately
    if estop_bit == 0:
        estop_publisher.publish(0)
    elif estop_bit == 1:
        estop_publisher.publish(1)
        current_estop_bit = 1
    
    while not rospy.is_shutdown(): 
        # Read the digital values from the pins
        vals = global_lines.get_values()
        estop_bit = vals[0]
        battery_bit1 = vals[1]
        battery_bit2 = vals[2]
        battery_bit3 = vals[3]

        print("estop: ", estop_bit)
        print("bit1: ", battery_bit1)
        print("bit2: ", battery_bit2)
        print("bit3: ", battery_bit3)

        battery_bits = [battery_bit1, battery_bit2, battery_bit3]

        if estop_bit == 1 and current_estop_bit == 0:
            current_estop_bit = 1
            estop_publisher.publish(1)

        if estop_bit == 0 and current_estop_bit == 1:
            current_estop_bit = 0
            estop_publisher.publish(0)

        # Convert the bits to a decimal number
        num = int("".join([str(b) for b in battery_bits]), 2)
        value = 0.0

        # Check which scenario has occurred
        if num == 0:
            value = 0.0
        elif num == 1:
            value = 0.125
        elif num == 2:
            value = 0.25
        elif num == 3:
            value = 0.375
        elif num == 4:
            value = 0.5
        elif num == 5:
            value = 0.625
        elif num == 6:
            value = 0.75
        elif num == 7:
            value = 1

        battery_percentage_publisher.publish(value)

        if value == 0.0:
            number_of_low_battery_detections = number_of_low_battery_detections + 1
            if (number_of_low_battery_detections > 30):
                #shutdown()
                print("Would shut down if activated")
        else:
            if (number_of_low_battery_detections > 0):
                number_of_low_battery_detections = number_of_low_battery_detections - 1

        rate.sleep()

if __name__ == "__main__":
    main()
