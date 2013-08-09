Torcs
=====

Driver for torcs

Requirements (tested on ubuntu 11.04 +)

  - freeglut3-dev
  - libplib-dev
  - libopenal-dev
  - libalut-dev
  - libvorbis-dev
  - libxmu-dev
  - libxxf86vm-dev
  
  if there is a problem linking musicplayer library (during make), copy this file in the torcs source and patch it with < patch -p1 -i torcs-1.3.4-fixes.patch > .

  link : http://sourceforge.net/mailarchive/attachment.php?list_name=torcs-users&message_id=4789A1A7-4FEE-4DF8-AD32-3F1C9D080FC9%40web.de&counter=1

- install torcs 1.3.4+ < http://torcs.sourceforge.net/ > 
  using this guide :   < http://www.berniw.org/tutorials/robot/tutorial.html > (mandatory) (check the environments)

- lemon graph library 1.2.3+ < http://lemon.cs.elte.hu/trac/lemon > (mandatory)

- gnuplot < http://www.gnuplot.info/ > (Optional, if you want to print plot of the track / car)

The link provided above (berniw) contanins a robot tutorial.

The folder /Frassi is the driver, you have to copy in torcs/src/drivers and then compile and install with make, make install.

Simply go to /torcs/src/drivers and then clone the repository.

The file variables.h contains the path where the driver log things or search for the files

If you want to print in a fast way the logs you can use the command for gnuplot which are in /frassi/gnuplot.txt





