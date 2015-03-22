# Introdução #

O [player](http://playerstage.sourceforge.net/) é uma biblioteca de controle de robôs. Esta página ensina a instalá-lo no Ubuntu (ao menos no 9.04), o que não é tão simples como se pode imaginar. O Stage é o programa de simulação.


# Instruções #

## Player ##
A parte complicada é saber quais dependências se deve instalar. [Esse cara](http://www.control.aau.dk/~tb/wiki/index.php/Installing_Player_and_Stage_in_Ubuntu) descobriu que a seguinte linha resolve nossos problemas:

```
sudo apt-get install autotools-dev build-essential cpp libboost-dev libboost-thread-dev libboost-thread1.33.1 libboost-signals-dev libboost-signals1.33.1 libltdl3 libltdl3-dev libgnomecanvas2-0 libgsl0 libgsl0-dev libgtk2.0-dev libjpeg62-dev libtool swig
```

Depois disso, [baixe](http://sourceforge.net/projects/playerstage/files/) a versão mais recente do Player e siga as instruções da página do Player:

```
# Uncompress and expand the tarball:
$ tar xzvf player-<version>.tgz

# `cd' into Player's source directory:
$ cd player-<version>

# To configure Player with default settings:
$ ./configure

# Compile Player:
$ make

# Install Player. By default, Player will be installed in /usr/local so you need to become root for this step. Remember to return to your normal user ID afterwards.
$ make install
```

## Stage ##
para instalar o stage:

```
sudo apt-get install cmake
sudo apt-get install libfltk1.1-dev libfltk1.1
sudo apt-get install libglu1-mesa-dev

no ~/.bashrc: LD_LIBRARY_PATH=/usr/local/lib
```