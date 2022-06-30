# Kontrolovanie robota jupiter cez obrazovku

## o kode 
Program je napisane v python jazyku a je tam naprogramovana komunikacia medzi modulmi vramci systemu ros.
V kode je naprogramovany pohyb robotickeho ramena a pohyb robota. Je pritom pouzity senzor lidar a je ovladetelny dotikovou obrazovkou.

## Ako naistalovat package
presunte patrik_shovcase subor do catkin_ws/src priecinka
## Ako spustit robota
zapnite napajenie.
![napajanie](https://user-images.githubusercontent.com/33252560/176631225-c3bc9f43-f69f-48af-ac9b-58d00f1a71bc.jpg)
spustite pocitac
![pocitac](https://user-images.githubusercontent.com/33252560/176631252-8e9b3df5-c819-44a2-8a04-b211a2a93060.jpg)

## Ako spustit program
![jupiter_bezi](https://user-images.githubusercontent.com/33252560/176631163-2d298ca6-b772-43d4-9838-412c8aed450a.jpg)

zapnite napajanie do robotickej ruky (Varovanie: ruka sa vystrie do vzpriamenej polohy ako je naobrazku dajte pozor aby netrafila display)
![napajanie_ramena](https://user-images.githubusercontent.com/33252560/176631236-86f6b43f-70ce-417b-8038-f0ffe9a2e0ea.jpg)

a odistite cerveny gombik ktore posunie napajanie do motorov.
otvorte terminal aj spustite tento kod.
```
roslaunch patrik_showcase demo.launch
```
## viac o programe
program je interaktivne menu vdaka ktoremu je mozne porgramovat robota.
pri spusteni
![menu](screenshot_aplikacie.png)

![jupiter](https://user-images.githubusercontent.com/33252560/176630987-dfa01345-dddb-493c-86fa-2aa726ef8cd9.jpg)

![napajanie](https://user-images.githubusercontent.com/33252560/176631225-c3bc9f43-f69f-48af-ac9b-58d00f1a71bc.jpg)
![napajanie_ramena](https://user-images.githubusercontent.com/33252560/176631236-86f6b43f-70ce-417b-8038-f0ffe9a2e0ea.jpg)
![pocitac](https://user-images.githubusercontent.com/33252560/176631252-8e9b3df5-c819-44a2-8a04-b211a2a93060.jpg)




This folder contains application for demonstration of Jupiter capabilities.

(c) Patrik Homola, FMFI UK, 2022

Links

http://learn.turtlebot.com/

