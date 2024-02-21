# Flying Bear Defense System (FBDS)

🛡️📡🚀 Embedded Software for the Internet of Things course group project

## Getting Started

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

![Bear riding a rocket](https://github.com/davidepaci/fbds/assets/23656588/97df4885-5722-4dcb-8542-59ddd7192674)
![Chad bear](https://github.com/davidepaci/fbds/assets/23656588/11ce18b9-2380-4eb8-9651-2d43b3532342)
![Parachuting bear](https://github.com/davidepaci/fbds/assets/23656588/fb91147f-69b8-4179-96c8-fbb16c35a385)
