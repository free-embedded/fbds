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

![Pasted image 20231106143805](https://github.com/davidepaci/fbds/assets/23656588/97df4885-5722-4dcb-8542-59ddd7192674)
![_1eee0d3d-611f-4487-a1d5-a8af42a13fd6](https://github.com/davidepaci/fbds/assets/23656588/11ce18b9-2380-4eb8-9651-2d43b3532342)
![_db5033e6-90c8-4518-ba36-977d305eed01](https://github.com/davidepaci/fbds/assets/23656588/fb91147f-69b8-4179-96c8-fbb16c35a385)
