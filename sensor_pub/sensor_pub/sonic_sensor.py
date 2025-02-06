import rclpy  #Import the ROS 2 Python client library
from rclpy.node import Node  #Import the Node class from the ROS 2 library
from std_msgs.msg import Float32, Bool  #Import standard message types for ROS 2
from gpiozero import DistanceSensor, Button  #Import sensor classes from the gpiozero library

class SensorPublisher(Node):
    def __init__(self):
        #Initialize the ROS 2 Node with the name SensorPublisher
        super().__init__('SensorPublisher')  
        
        #Initialize the distance sensor connected to GPIO pins 9 and 10
        self.sensor = DistanceSensor(9, 10)
        #Initialize the button (bumper) sensor connected to GPIO pin 4, without a pull-up resistor
        #self.bumper = Button(4, pull_up=False)
        
        timer_period = 0.1  #Set the timer period for sensor data publishing (in seconds)

        #Create a publisher for the sonar sensor data
        self.sonar_publisher = self.create_publisher(Float32, 'SonarSensorInfo', 1)
        #Create a timer that calls the 'publish_sonar' method at regular intervals
        self.sonar_timer = self.create_timer(timer_period, self.publish_sonar)

        #Create a publisher for the bumper sensor data
        #self.bumper_publisher = self.create_publisher(Bool, 'BumperSensorInfo', 1)
        #Create a timer that calls the 'publish_bumper' method at regular intervals
        #self.bumper_timer = self.create_timer(timer_period, self.publish_bumper)
        
    #Method to publish sonar sensor data
    def publish_sonar(self):
        msg = Float32()  #Create a new Float message
        msg.data = self.sensor.distance  #Message distance mesuared by sensor
        self.sonar_publisher.publish(msg)  #Publish the message
        self.get_logger().info('Publishing: "%f"' % msg.data)  #Log the published data

    #Method to publish bumper sensor data
    #def publish_bumper(self):
    #    msg = Bool()  #Create a new Bool message
    #    msg.data = self.bumper.is_pressed  #Set the message to the state of the bumper pressed or not
    #    self.bumper_publisher.publish(msg)  #Publish the message
    #    self.get_logger().info('Publishing: "%s"' % msg.data)  #Log the published data
        
    #Clean up method when the object is deleted
    def __del__(self):
        self.sensor.close()  #Close the distance sensor
    #    self.bumper.close()  #Close the bumper sensor

def main():
    rclpy.init()  #Initialize the ROS 2 Python client library
    sensorPublisher = SensorPublisher()  #Create an instance of the SensorPublisher class

    try:
        rclpy.spin(sensorPublisher)  #Keep the node running until interrupted
    except KeyboardInterrupt:
        print("Interrupted by keyboard") #Handle the keyboard interrupt by printing a message
    finally:
        sensorPublisher.sensor.close() #Clean up the sensors
    #    sensorPublisher.bumper.close() #Clean up the sensors
        sensorPublisher.destroy_node() #Destroy the node explicitly
        rclpy.shutdown()  #Shutdown the ROS 2 Python client library

if __name__ == '__main__':
    main()  #Call the main function if this script is executed