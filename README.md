#Brachialbiber
=============
##Allgemein:
Dieser Code war dazu gedacht, mittels eines EV3 von Lego den Roborace 2014 der Technischen Kybernetik in Stuttgart zu gewinnen. 
##Aufgabenstellung:
<blockquote>
Die diesjährige ROBORACE-Aufgabe ist an das Kinderspiel „Eierlauf“ angelehnt. Das Ziel ist es, mit 
einem Roboter, der eine vorgegebene Tragevorrichtung hat (siehe Anhang), einen Ball möglichst 
schnell über einen Hindernisparcours zu balancieren, ohne dass dieser auf den Boden fällt. Der 
Parcours besteht aus mehreren Berg- und Tal-Streckenteilen, die es zu überwinden gilt und unter 
keinen Umständen ausgelassen werden dürfen. Als Orientierungshilfe ist eine schwarze Linie auf der 
Strecke aufgebracht. Die Schwierigkeit der Aufgabe besteht darin, dass die Lage des „Löffels“ ständig 
an den Streckenverlauf angepasst werden muss, damit der Ball nicht aus der Halterung fällt. 
Idealerweise wird die Tragevorrichtung so geregelt, dass sie sich stets in einer horizontalen Lage 
befindet.  
</blockquote>
##Probleme und Herausforderungen:
Das Anpassen der Konstanten für unseren LineFollower-PID ist sehr mühsam und aufwendig. Außerdem spielt die Konstruktion nach unserer Erfahrungen eine nicht unwesentliche Rolle. (Abstand Farbsensor etc.). 
Der Gyro funktionierte soweit, hat allerdings manchmal (zu oft) einen Drift.
Unter Umständen lohnt es sich, den Roboter mit Codeblocks anstatt via Lejos zu programmieren, da Lejos soweit wir wissen noch einige Bugs hat. Außerdem gab es einige sehr erfolgreiche Gruppen die nur mit Codeblocks arbeiteten.

