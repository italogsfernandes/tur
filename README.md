# EnTulio (ElSarrador)

Projeto para a competição de robôs minisumô, a [sumocup na ufu](http://www.lasec.feelt.ufu.br/sumocup).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

There are 6 lines for the final robot, from the first option to the sixth:

1. **Made in China Robot**: The idea is to assemble a personal sumobot using this chinese motors.

![](/Docs/Pictures/motors_encoders_wheel.JPG)

2. Stepper option: A sumobot using stepper motors. **Decide with Motor** **Quickly**

3. ElSarrador: The first test.

![](/Docs/Pictures/sarrador_v001.jpg)

4. Feijuca: Packet with 500g of beans.

![](/Docs/Pictures/feijao.jpg)

5. Ronaldo's Robot: A adapted line follower, just for tests and for fun.

![](/Docs/Pictures/chassi-acrilico.jpg)

6. Pololu Zumo: We have not  enough money, but we think it's a bealtiful robot. If you are reading this and you have money, we will be thankfull for a robot-donation.

![](/Docs/Pictures/pololu_zumo.jpg)

### In this repository

There are a lot of folders and files, but there are nothing in most of then :( - we need to work.

- [ ] Firmware: Arduino Codes.
  - [ ] Main: The main arduino code.
  - [ ] Other folders: The code for using and testing each part of the project.
    - [ ] Buzzer_and_PushButton: To Do.
    - [ ] DC_Motor: To Do.
    - [ ] Distance_Sensor: To Do.
    - [ ] Inertial_Sensor: **Working on it.**
    - [ ] LCD_Display: To Do.
    - [ ] Line_Sensors: To Do.
    - [ ] Servo_Motor: To Do.
    - [ ] Sttepers_Motors: To Do.
- [ ] Hardware: PCB, schematic and 3D models.
  - [ ] 3D: 3D model of the final robot, including some parts for 3d printing.
    - [x] Libraries: Useful 3d models of arduino modules, arduinos, mechanical structures and eletronic devices.
    - [ ] 3D Prototype: **Working on it.**
    - [ ] 3D Print Files: **Working on it.**
  - [ ] Circuit: PCB and schematic.
    - [ ] Schematic: To Do.
    - [ ] PCB layout: To Do.
    - [ ] Fritzing scheme: **Working on it.**

## Development Environment

* Firmware - Arduino:
  *  [Platomformio](https://atom.io/packages/platomformio) - Atom integration with PlatformIO (for building arduino files,but arduino IDE can also be used.).
  * [Arduino ID](www.arduino.cc) - If you prefer.
  * [Library for Inertial Sensor] (https://github.com/jrowberg/i2cdevlib) - Download and place the folders Arduino/MPU6050, Arduino/HMC5883L and Arduino/i2cdevlib in the right place.
* Hardware - 3D:
  * Cubex.
  * SketchUp, AutoCad, Blender - Choosing...
  * Cubex and 3dBuider - Needed for 3dPrinting.
* Hardware - Circuit:
  * Fritzing.
  * Eagle, Proteus, Altium, Orcad - Choosing...

## Versioning

There aren't actual versions, the nearest is the "saco de feijão".

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## More Info

* A nice [video](https://www.youtube.com/watch?v=ABleYAFDfmY).
![](/Docs/Pictures/playgroundvideo.JPG)

* [Rules](http://www.lasec.feelt.ufu.br/sumocup).

![](/Docs/Pictures/rules.JPG)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

```
"THE BEERWARE LICENSE" (Revision 42):
Italo Fernandes, Adrielle Nazar and Ronaldo Sena wrote this code. As long as you retain this
notice, you can do whatever you want with this stuff. If we
meet someday, and you think this stuff is worth it, you can
buy us a beer in return.
```
## Authors

* **Italo Fernandes** - https://github.com/italogfernandes - italogsfernandes@gmail.com

* **Adrielle Nazar** - adrielle.nazar@hotmail.com

* **Ronaldo Sena** - https://github.com/ronaldosena - ronaldo.sena@outlook.com

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.
