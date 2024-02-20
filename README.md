# Flying Bear Defense System (FBDS)

🛡️📡🚀
The Flying Bear Defense System (FBDS) consists of a fully controllable turret made of a TI MSP432P401R Launchpad, a TI Booster Pack MK II, 3 servos, a pan-and-tilt turret and a Nerf blaster.
It also features an automatic mode via a webcam mounted on it, powered by OpenCV. Said program tracks an ArUco marker and tells an ESP32 which direction to move to, which in turn forwards said direction to the MSP432P401R.

This was a group project for the _Embedded Software for the Internet of Things_ course at University of Trento.

![Removal-438](https://github.com/free-embedded/fbds/assets/23656588/b18a7514-447f-408d-ae5a-e0ea6fe7d8b8)

## Table of Contents
- [Flying Bear Defense System (FBDS)](#flying-bear-defense-system-fbds)
  - [Table of Contents](#table-of-contents)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Involved](#software-involved)
  - [Getting Started](#getting-started)
  - [How to Use](#how-to-use)
  - [Project Presentation](#project-presentation)
  - [Team Contributions](#team-contributions)

## Hardware Requirements
- MSP432P401R (TI)
- Booster Pack MK II (TI)
- Nerf Elite 2.0 Ace SD-1 blaster
- Servos: 1x DGSERVO S05NF STD, 2x SM-S4303R
- Pan and Tilt Turret
- Breadboard
- ESP32
- Webcam for target detection

## Software Involved
- C program utilizing DriverLib and graphics library for MSP432
- C++ program for communication between PC and MSP432 via ESP32
- Python script with OpenCV for target detection through webcam

## Getting Started
Connect the following pins in the breadboard:
- Pin 2.5 -> Servo 1 (X axis)
- Pin 2.4 -> Servo 2 (Y axis)
- Pin 2.6 -> Servo 3 (Trigger)
- Pin 3.2 (UART RX) -> ESP32 Pin 16 (TX)
- Pin 3.3 (UART TX) -> ESP32 Pin 17 (RX)

Connect the webcam, the ESP32, and the MSP432 to the PC.

Load the Nerf gun with the dart and pull back the spring.

Run `thing3.py` (requires `pyserial` and `opencv-python`)

Then run the following commands:

```sh
> git clone https://github.com/free-embedded/fbds.git
> cd fbds
> make
> openocd -f board/ti_msp432_launchpad.cfg
```

In another terminal (preferably in the project folder, so the paths are shorter):

```sh
> arm-none-eabi-gdb
> (gdb) target remote :3333
> (gdb) load ./out/fbds.out
> (gdb) continue
```

## How to Use
The system will start in automatic mode. Press the top button to switch to manual mode. In manual mode, use the joystick to control the turret on both axes and press the bottom button to shoot the dart.

## [Project Presentation](https://www.canva.com/design/DAF9WZTbPrw/5qKn4lTqf2faBvDL3ldvow/edit?utm_content=DAF9WZTbPrw&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

## Team Contributions
| Task              | Contributors             |
|-------------------|--------------------------|
| Servo movement    | Simone De Carli, Davide José Paci |
| PWM               | Simone De Carli          |
| Git Maintenance   | Simone De Carli, Marco Toniolo |
| Code Structure    | Simone De Carli          |
| Hardware Setup    | Mattia Maci              |
| Project Video     | Mattia Maci              |
| Automatic Mode    | Davide José Paci         |
| Project Outline   | Davide José Paci         |
| Project README    | Davide José Paci         |
| UART              | Davide José Paci         |
| Buzzer Music      | Marco Toniolo            |
| LCD Display       | Marco Toniolo            |
| Makefile          | Marco Toniolo            |

![Bear riding a rocket](https://github.com/davidepaci/fbds/assets/23656588/97df4885-5722-4dcb-8542-59ddd7192674)
![Chad bear](https://github.com/davidepaci/fbds/assets/23656588/11ce18b9-2380-4eb8-9651-2d43b3532342)
![Parachuting bear](https://github.com/davidepaci/fbds/assets/23656588/fb91147f-69b8-4179-96c8-fbb16c35a385)