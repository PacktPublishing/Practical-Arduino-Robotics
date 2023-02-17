# Practical Arduino Robotics

<a href="https://www.packtpub.com/product/unlocking-the-power-of-arduino-in-diy-robots/9781804613177"><img src="https://m.media-amazon.com/images/I/4172bzAJNOL.jpg" alt="Book Name" height="256px" align="right"></a>

This is the code repository for [Practical Arduino Robotics](https://www.packtpub.com/product/unlocking-the-power-of-arduino-in-diy-robots/9781804613177), published by Packt.

**A hands-on guide to bringing your robotics ideas to life using Arduino**

## What is this book about?
Every robot needs a “brain,” and the Arduino platform provides an incredibly accessible way to bring your Arduino robot to life. Anyone can easily learn to build and program their own robots with Arduino for hobby and commercial uses, making Arduino-based robots the popular choice for school projects, college courses, and the rapid prototyping of industrial applications!
Unlocking the Power of Arduino for DIY Robots is a comprehensive guide that equips you with the necessary skills and techniques that can be applied to various projects and applications, from automating repetitive tasks in a laboratory to building engaging mobile robots.
Building on basic knowledge of programming and electronics, this book teaches you how to choose the right components, such as Arduino boards, sensors, and motors, and write effective code for your robotics project, including the use of advanced third-party Arduino libraries and interfaces, such as Analog, SPI, I2C, PWM, and UART.

This book covers the following exciting features: 
* Understand and use the various interfaces of an Arduino board
* Write the code to communicate with your sensors and motors
* Implement and tune methods for sensor signal processing
* Understand and implement state machines that control your robot
* Implement feedback control to create impressive robot capabilities
* Tune, debug, and improve Arduino-based robots systematically

If you feel this book is for you, get your [copy](https://www.amazon.com/Unlocking-Power-Arduino-Robots-hands-ebook/dp/B0BNNX7DD8) today!

<a href="https://www.packtpub.com/?utm_source=github&utm_medium=banner&utm_campaign=GitHubBanner"><img src="https://raw.githubusercontent.com/PacktPublishing/GitHub/master/GitHub.png" alt="https://www.packtpub.com/" border="5" /></a>

## Instructions and Navigations
All of the code is organized into folders. For example, Chapter03.

The code will look like the following:
```
// Include the Servo library.
#include <Servo.h>

// Analog input pin.
const int input_pin = A5;
// Servo output pin. Must be a PWM pin.
const int servo_pin = 3;
// Instantiate a Servo object called servo.
Servo servo;

void setup() {
  // Attach the servo object to the servo pin.
  servo.attach(servo_pin);
}

void loop() {
  // Read the potentiometer.
  int input = analogRead(input_pin);
  // Map the analog value from 0 to 1023 to a pulse width
  // between 1000 and 2000.
  // The suffix _us stands for microseconds.
  int pulse_width_us = map(input, 0, 1023, 1000, 2000);
  // Set the servo PWM pulse width to that value.
  servo.writeMicroseconds(pulse_width_us);
}

```

**Following is what you need for this book:**
If you’re excited about robotics and want to start creating your own robotics projects from the hardware up, this book is for you. Whether you are an experienced software developer who wants to learn how to build physical robots, a hobbyist looking to elevate your Arduino skills to the next level, or a student with the desire to kick-start your DIY robotics journey, you’ll find this book very useful. In order to successfully work with this book, you’ll need basic familiarity with electronics, Arduino boards and the core concepts of computer programming.

With the following software and hardware list you can run all code files present in the book (Chapter 1-14).

### Software and Hardware List

| Chapter  | Software required                                                       | OS required                       |
| -------- | ------------------------------------------------------------------------| ----------------------------------|
| 1-14     | Arduino IDE 2.0.3 (or higher)                                           | Windows, Mac OS X, and Linux (Any)|
| 1-14     | Arduino Uno or Mega2560                                                 |                                   |

We also provide a PDF file that has color images of the screenshots/diagrams used in this book. [Click here to download it](https://packt.link/THXao).

### Related products <Other books you may enjoy>
* Learn Robotics Programming - Second Edition [[Packt]](https://www.packtpub.com/product/learn-robotics-programming-second-edition/9781839218804) [[Amazon]](https://www.amazon.com/Learn-Robotics-Programming-AI-enabled-autonomous/dp/1839218800)

* Raspberry Pi Pico DIY Workshop [[Packt]](https://www.packtpub.com/product/raspberry-pi-pico-diy-workshop/9781801814812) [[Amazon]](https://www.amazon.com/Raspberry-Pico-DIY-Workshop-automation/dp/1801814813)

## Get to Know the Author
**Lukas Kaul**
He is a robotics Research Scientist, currently working at the Toyota Research Institute in Silicon Valley, where he develops mobile manipulation technologies to support people in their homes and in their workplace. Throughout his career he has worked on projects as diverse as humanoid robots, aerial robots and mobile manipulation systems. A maker at heart, Lukas has been using Arduino technology extensively for more than a decade in countless side-projects, ranging from mapping systems to self-balancing robots. Lukas is passionate about teaching robotics with Arduino to inspire and empower anyone who wants to enter the exciting field of robotics. He holds a PhD degree from the Karlsruhe Institute of Technology, Germany.
