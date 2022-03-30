# Convolutional Neural Network für Bildklassifizierung

In diesem Notebook wird ein CNN Implementiert zur Klassifizierung von Bildern. Die Theorie dazu wird ausführlich im [Storyboard](https://www.aiav.technikum-wien.at/ai-anwenden) beschrieben. 

![0](./visu/example_0.JPG) 
![1](./visu/example_1.JPG) 
![2](./visu/example_2.JPG) 

## Bibliotheken

Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. Wie das genau funktioniert ist in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) auch beschrieben.  

Die Implementierung des [CNN Models](https://www.tensorflow.org/tutorials/images/classification) ist mittels [Tensorflow](https://www.tensorflow.org/) in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Aufgrund von unterschiedlichen Strukturen von Tensorflow ist es wichtig, dass Tensorflow 2.8.0 installiert ist. Dies kann entweder mittels der requirements.txt Datei gemacht werden oder manuel mit  **>> _pip3 install tensorflow==2.8.0_ <<**

## Ordnerstruktur

Das CNN Model benötigt - wie auch die anderen Modelle - zum Trainieren einen Datensatz. Wir verwenden dazu einfach Ordner mit verschiedenen Bildern. Im Ordner [data](./data) sind die ganzen Trainingsdaten abgelegt. Da wir "nur" eine Klassifizierung machen, können wir die Bilder Labeln indem wir den Ordnernamen so benennen wie auch unsere Klasse heißen soll. So kann der Code auch einfach angepasst werden, sollten eigene Bilder verwendet werden. Da wir versuchen die Zahlen '0' bis '9' zu klassifizieren heißen so auch die Unterordner. 

Der Ordner demo beinhaltet von jeder Zahl ein Test bild, welches wir nach dem Trainieren des Modells schnell einsetzen können um zu sehen ob unser Modell funktioniert.



## Ergebnisse
 
![Webcam](./Webcam_Implementation.gif) 




## Was nun? 


Hierzu können wir den CNN-Detector Use-Case empfehlen ([Coming soon](https://www.aiav.technikum-wien.at/))



Falls dieses zu komplex oder Hardware intensiv ist, können wir die etwas rechenfreundlicheren Klassifizierungen empfehlen welche wir in den unten aufgelisteten Use-Cases behandelt haben. Vielleicht passen diese besser für Ihren Anwendungsfall? 

#### logistische Regression </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) </br>
### Support Vector Machine </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Support_Vector_Machine_fuer_Bildklassifizierung) </br>
#### k-Neares Neighbour </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/kNearest_Neighbor_fuer_Bildklassifizierung) </br>
#### Random Forest </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Random_Forest_fuer_Bildklassifizierung)


## Weitere externe Informationen/ Quellen

[CNN Model](https://www.tensorflow.org/tutorials/images/classification) </br>
[Konvertieren OpenCV Mat zu Tensorflow](https://stackoverflow.com/questions/40273109/convert-python-opencv-mat-image-to-tensorflow-image-data/40273815)</br>