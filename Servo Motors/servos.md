# Servo Motors

> By : Shubhankar Sharma

## What are servo motors?

<p align="center">
<image src="thumb.jpg"
width="500px"
position="center">
</p>

The term servo was first used by **_Joseph Farcot_** in 1859, who implemented a feedback mechanism to assist in steering a ship with steam to control the rudders. Servo motors are electromechanical devices and rotary or linear actuators that rotate and push machine parts with precision and produce torque and velocity based on the supplied current and voltage. They are specialized for high precision and response positioning. They are easily controlled, and their starting torque is high, so they are usually preferred industrially.

<p align="center">
<image src="joseph.jpg"
width="250px"
position="center">
</p>
<p align="center">Jean Joseph Léon Farcot or Joseph Farcot</p>

A servo motor is part of a servo mechanism consisting of three key elements – a motor, a feedback device, and control electronics. A standard induction motor is rated in kW or HP, but a servo motor is rated at kg/cm; this is so because it shows how much load its shaft can carry per cm, or it represents the motor’s toque for a pulley at the radius of 1cm, and it gets weaker as radius increases. Usually, a servo motor is rated at 3 kg/cm, 6 kg/cm, and 12 kg/cm. These motors use a reduction gearbox which lowers the RPM (rotation per minute) and provides high torque. In a servo motor, speed is determined by the frequency of the applied voltage and the number of magnetic poles. The servo motor has a rotation detector (encoder) mounted on the back shaft side to detect the rotor’s position and speed. This enables high resolution, high-response positioning operation.Joseph Facort

<BR>

## The types of servo motors

A servo motor can be brushed or brushless and can be of any size. Basically, a servo motor can have two types of current input AC or DC :<br>
An AC servo motors can handle higher current surges and are thus used in heavy industrial machinery. Still, a DC servo motor is best suited for small applications which has excellent controllability and feedback.
<br>
But generally, there are three basic types of servos which are used : <br>

1. A **Positional Rotation** servos rotate 180 degrees. They also have stops in the gear mechanism to protect the output shaft from over-rotating.
2. A **Continuous Rotation** servo motor is a servo that does not have a limit on its range of motion. Instead of having the input signal determine which position the servo should rotate to, the continuous rotation servo relates the input to the speed of the output and direction. The limitless motion of these motors enables them to move in both CW and CCW directions.
3. A **Linear Servos** use a rack and pinion mechanism to change their output. The rack and pinion convert rotary motion to linear motion.
   <br>
   <br>

## Controlling of Servo Motors

Servos are controlled by Pulse width modulation (PWM) that sends an electric pulse of variable width to the motor. With PWM, there is a minimum pulse, maximum pulse, and a repetition rate. The rotor will turn to the desired position based on the duration of the pulse. When servos are commanded to move, they move to the position and hold it. The below graph is the pulse width modulation graph for Servo Motors:
<br>
<br>

<p align="center">
<image src="Pulse.png"
width="500px"
position="center">
</p>
<br>
<br>

The Analog and digital servo motors look exactly the same. The difference is in the way they signal and process information.
Analog Servos operate based on voltage signals that come through the pulse width modulation (PWM). When an analog servo is at rest, the PWM is essentially off unless you transmit some action. Producing torque from the resting mode makes the initial reaction time sluggish. This delay in torque isn’t ideal for advanced applications.
Digital Servos use a small microprocessor to receive and direct action at high-frequency voltage pulses. The digital servo sends nearly six times the amount of pulses an analog signal does. These faster pulses provide consistent torque for quicker and smoother response times. It’s important to note that faster pulses require more power emission from the motor. The below graph is for analog servo motors vs digital servo motors pulse width modulation.
<br>
<br>

<p align="center">
<image src="cycle.png"
width="500px"
position="center">
</p>
<br>
<br>

## Application of Servo Motors

As a servo motor capable of accurate rotation angle and speed control, it can be used for various equipment.

- Tuning Free
- Compact and High Power
- Wide Variable Speed Range
- Standard or Planetary Geared Type
- Electromagnetic Brake Types

Generally, there are several applications of servo motors in our world, some of the points related to above topics are :

<p align="center">
<image src="robotic.jpg"
width="200px"
position="center">
</p>

1. Robotics: robotic arms and their movements needs precision so servos are attached to every joints of the robot giving it precise angle and movements.
<br>
<br>
<p align="center">
<image src="camera.png"
width="200px"
position="center">
</p>

2. Camera Auto Focus: A highly precise servo motor built into the camera corrects a camera’s lens to sharpen out-of-focus images.
<br>
<br>
<p align="center">
<image src="belt.jpg"
width="200px"
position="center">
</p>

3. Conveyor Belts: Servo motors move, stop, and start conveyor belts carrying product along to various stages, for example, in product packaging/bottling, and labeling.
<br>
<br>
<p align="center">
<image src="antenna.jpeg"
width="200px"
position="center">
</p>

4. Antenna Positioning: Servo motors are used on both the azimuth and elevation drive axis of antennas and telescopes such as those used by the National Radio Astronomy Observatory (NRAO)
   <br>

## Working and Structure of Servo Motors

### Working of Servo Motors :

It works as a part of closed loop control system (feedback system) providing torque and velocity as commanded from a servo controller utilizing a feedback device/position sensors (Usually encoders for AC servo motors which are used in CNC (Computer Numerical Control) and VMC ( Vertical Machining Center) machines and Potentiometers in DC servo motors) to close the loop. The feedback device supplies information such as position ,current, velocity to the servo controller, which adjusts the motor action depending on the commanded parameters. The below represents the block diagram of working of the feedback system.

<br>
<p align="center">
<image src="feedback.png"
width="500px"
position="center">
</p>
<br>
<br>

1. The **RYB** or the power input is supplied to VFD ( variable frequency drive) which is the **controller** or servo driver which commands the working of the motor.
1. The **controller** is next connected to the **motor**.
1. The **motor** is then connected to the **gears** (reduction gearbox) which provides low RPM and high torque which is next connected to the **O/P** or the shaft.
1. The **gearbox** is also connected to **position sensors** or the feedback system which provides the information about the velocity, current and position which it provides to the control unit or the controller which is next processed by the control unit to either stop or provide the input to the motor.
   <br>
   <br>
   Usually when the motor works on low RPM such as 5 RPM, 10 RPM the motor stops at exact same point when the controller cut-offs the power but in industrial motors which works on 1,500 RPM and 3,000 RPM the stopping of motor or shaft at same position is not possible so the system then uses the braking system which is placed after the reduction gearbox so as to stop the motor/shaft from moving from the point when the controller cuts-off the power.
   <br>

### Structure of Servo Motor :

The below figure shows the structure of an AC servo motors:
<br>

<p align="center">
<image src="struct.jpg"
width="500px"
position="center">
</p>
<br>
<br>
   
1. **Stator** From the position of the rotor, a rotating magnetic field is created to efficiently generate torque.
2. **Winding** Current flows in the winding to create a rotating magnetic field.
3. **Bearing** Ball Bearing
4. **Shaft** This part transmits the motor output power. The load is driven through the transfer mechanism (such as the coupling).
5. **Rotor** A high-function rare earth or other permanent magnet is positioned externally to the shaft.
6. **Encoder** The optical encoder always watches the number of rotations and the position of the shaft.
7. **Encoder Cable** transmits the data to the servo driver or the controller
8. **Motor Cable** is used as the power supply to the motor. Here we usually get 4 pins which are namely RYB and the fourth pin is the earthing. Here all 4 pins are connected to Servo driver.

The encoder is a sensor for detecting the speed and position of the motor. Light from the light-emitting diode (LED) passes through a position detection pattern on the slit disk and is read by the light-receiving element. Dozens of photo transistors are integrated in the light-receiving element. All of the patterns for absolute position detection depends on the rotation angle of the encoder. The CPU is mounted on the encoder for analysis of the absolute position detection patterns. The current position data is transmitted to the servo driver via serial transmission.
<br>

These 4 pins motor cable is usually seen in industrial servo motors or heavy duty servo motors but in small servo motors or light duty servo motors such as SG90 which are used in arduino or other micro-controllers there is no such encoder cable as the potentiometer is attached internally to the the motor. The Signal lead or the PWM pin is there that is used as the input data to servo from the micro-controller. The input cable consists of 3 pins :

- Red wire → Positive lead / Power
- Brown or black wire → Negative lead / GND
- Yellow or orange wire → Signal lead / PWM
  <br>
  <br>
  <p align="center">
  <image src="sg90.png"
   width="500px"
   position="center">
   </p>

 <p align="center"> The above picture is of SG90 (9g) servo motors which are generally used with micro-controllers such as arduino , fritzig, raspberry pi. </p>
  <br>
  <br>

## Advantages and Limitations of using Servo Motors

Servo motors are boon to the industrial sector as well the general usage as it makes the machinery give output to the point but there are also some the limitations which make servos to be not used all the time.

### Advantages :

- Precision: Highly precise operations
- Speed: High speed and more torque in a small package
- Encoder: Translates rotary or linear motion to a digital signal

Servo motors are known to always be frequent and work at the same pace. So, if a heavy load is placed on the motor, the driver will increase the current to the motor coil as it rotates the motor. This basically means that servo motors are expected to always be mechanically on point. And because of its precision, it allows companies to operate it at a high-speed pace.

### Disadvantages :

- High maintenance and operation cost.

When a machine using a servo is stopped, the motor continues to move back and forth one pulse, so it is not good if the machine or area is not suitable with vibrations.

---

## DIY Example of a Servo Motor :

We will now see how the servo motor works. We will create a simple example where we use PICO to rotate our servo motor from 0 to 180 degree and vice-versa. For this hardwares required are :

1. Arduino Board
1. Servo Motor (SG90)
1. Male-female wires
1. Bread-Board

Now, we will see the pinouts of servo motor’s:

- Red wire → Positive lead.
- Brown or black wire → Negative lead
- Yellow or orange wire → Signal lead

Now lets get started:

We will use pin 9 as the signal lead to servo motor for the controlling of servo motor.

- Connect the signal lead to pin 9 using the breadboard via male wires
- Now connect the +5V and GND pins to the (red and black wires respectively) to the servo motor using breadboard
- Now our connection is completed. Now lets getting to the coding part.
- Connect your Arduino board to the system/computer and upload the following code:

```
 #include <Servo.h>


   Servo myservo;     // create servo object to control a servo
   // twelve servo objects can be created on most boards

   int pos = 0;     // variable to store the servo position

void setup() {
      myservo.attach(9);     // attaches the servo on pin 9 to the servo object
}

void loop() {
      for (pos = 0; pos <= 180; pos += 1) {     // goes from 0 degrees to 180 degrees
                                              // in steps of 1 degree
   myservo.write(pos);     // tell servo to go to position in variable 'pos'
   delay(15);     // waits 15ms for the servo to reach the position
}
      for (pos = 180; pos >= 0; pos -= 1) {     // goes from 180 degrees to 0 degrees
   myservo.write(pos);     // tell servo to go to position in variable 'pos'
   delay(15);     // waits 15ms for the servo to reach the position
}
}
```

<br>

### Circuit Design :

<br>
  <br>
  <p align="center">
  <image src="circuit.png"
   width="500px"
   position="center">
   </p>

**Note** : Your connection of Arduino board to the system will work as the power input to the present setup.

> CODE LOGIC :
> We will use the “Servo.h” library that gives us the ability to use functions that will help us control the servo motor. The aim of this program is to move the servo motor shaft from angle 0 to angle 180 then return back from angle 180 to angle 0 again. We are using two for loops, one for increasing the motor shaft angle position (from 0 to 180), and the second loop is for returning back the motor shaft to 0 (from 180 to 0), and the servo's current angle will be continuously printed on the serial monitor.

<br>

---

## Projects related with servo motors :

There are various projects which you can make by using servo motors from this link [here](https://create.arduino.cc/projecthub/projects/tags/servo?page=1) , but I want to list some listed projects which you should must try :

- [Robotic Arm](https://create.arduino.cc/projecthub/circuito-io-team/robotic-arm-from-recycled-materials-7e318a?ref=tag&ref_id=servo&offset=2)
- [DIY Hand Sanitizer Dispenser](https://create.arduino.cc/projecthub/MissionCritical/diy-hand-sanitizer-dispenser-using-arduino-143de1?ref=tag&ref_id=servo&offset=21)

* [Obstacle Avoiding Rover](https://create.arduino.cc/projecthub/mindhe_aniket/obstacle-avoiding-robot-765e39?ref=tag&ref_id=servo&offset=27)
* [Quadruped Robot](https://create.arduino.cc/projecthub/Varun2905/quadruped-robot-5646bc?ref=tag&ref_id=servo&offset=52)
* [Automated Chrome Dino Game](https://create.arduino.cc/projecthub/diyelectronic/automated-dino-game-using-arduino-8ceccc?ref=tag&ref_id=servo&offset=45)

---

Thank YOU !!!
