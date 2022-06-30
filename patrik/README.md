# Kontrolovanie robota jupiter cez obrazovku

## o kóde 
Program je napísaný  v python jazyku a je tam naprogramovaná komunikácia medzi modulmi v rámci systému ROS.(Robot Orientating System má k dispozícii všetky základné prvky, ktoré pri tvorbe robotických aplikácii potrebujete, od ovládačov pre rôzne typy senzorov a hardwaru)
V kóde je naprogramovaný pohyb robotického ramena a pohyb robota. Je pritom použitý senzor 360 stupňový lidar (Lidar je rotujúci senzor ktorý vysiela infračervený luč a zachytáva jeho odraz. Slúži na zistenie vzdialenosti) a je ovládateľný dotykovou obrazovkou.

## Ako nainštalovať package
presuňte patrik_shovcase subor do catkin_ws/src priečinka.
## Ako spustiť robota
zapnite napajenie.
![napajanie](https://user-images.githubusercontent.com/33252560/176631225-c3bc9f43-f69f-48af-ac9b-58d00f1a71bc.jpg)
spustite počítač
![pocitac](https://user-images.githubusercontent.com/33252560/176631252-8e9b3df5-c819-44a2-8a04-b211a2a93060.jpg)

## Ako spustiť program
![jupiter_bezi](https://user-images.githubusercontent.com/33252560/176631163-2d298ca6-b772-43d4-9838-412c8aed450a.jpg)

zapnite napájanie do robotickej ruky (Varovanie: ruka sa vystrie do vzpriamenej polohy ako je na obrázku dajte pozor aby netrafila display)
![napajanie_ramena](https://user-images.githubusercontent.com/33252560/176631236-86f6b43f-70ce-417b-8038-f0ffe9a2e0ea.jpg)

a odistite červený gombík ktorý ovláda napájanie do motorov.
otvorte terminál aj spustite tento kód.
```
roslaunch patrik_showcase demo.launch
```
## viac o programe
Program je interaktívne menu vďaka ktorému je možne programovať robota.
![menu](screenshot_aplikacie.png)
Na ľavej strane sú príkazy ktoré pohybuje robotické rameno. Na pravej strane sú príkazy ktoré pohybujú robota. 
Tlačidlo stop zastaví robota na mieste. Tlačidlo „follow the wall“  spusti program kde robot pomocou lidaru bude sledovať po strane steny a bude sa vyhýbať prekážkam. Bohužiaľ nemám zaznamenané na videu to posledné z dôvodu pokazenia lidaru senzora v dobe keď som robota natáčal na video.  Tu je video demonštrácia.

[link na video](https://www.youtube.com/watch?v=hhut5zByEuA)




This folder contains application for demonstration of Jupiter capabilities.

(c) Patrik Homola, FMFI UK, 2022

Links

http://learn.turtlebot.com/
