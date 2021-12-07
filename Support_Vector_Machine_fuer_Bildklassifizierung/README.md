# Support Vector Machine

Hier befindet sich der [Beispielcode](./miniUsecase12_SVM.ipynb) für das Implementieren einer Support Vector Machine. Der Use-Case befasst sich mit der Klassifizierung von Bildern. Genauer gesagt, wollen wir herausfinden ob in einem Bild – beziehungsweise einem Kamerastream – sich ein Hammer befindet oder nicht. Diese Problemstellung wurde schon im vorherigen [Use-Case](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) behandelt. Allerdings befassen wir uns diesmal mit der angesprochenen SVM anstatt einer logistischen Regression. Die Theorie und der Aufbau ist im [Storyboard](12_Storyboard_SVM.pdf) genau beschrieben.


# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/stable/modules/svm.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. Wie das genau funktioniert ist in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) auch beschrieben.  

# Ordnerstruktur
Die SVM benötigt zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze Trainiert werden. Im Ordner [Tool_Data](./Tool_Data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo/SVM_Test.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild notiert ob das Model einen Hammer im Arbeitsbereich erkannt hat oder nicht. 

![Abbildung 1](demo/SVM_Test.gif)

# Was nun? 
In dem Use-Case haben wir uns mit der Klassifizierung von Bildern mittels der Support Vector Machine befasst. Wenn Sie weiteres Interesse an klassifizierungs Modellen haben, empfehlen wir folgende Use-Cases auf der Plattform. Der erste Link (logistische Regression) beschreibt das gleiche Problem wie in dem Use-Case allerdings mit einer logistischen Regression. Ebenso ist dort die Theorie zu der Umwandlung von einer Bildmatrix in einen Importvektor erklärt. Die anderen beiden Use-Cases sind ebenso nach demselben Schema aufgebaut. Sie haben allerdings jeweils ein anderes Model, welches die Klassifizierung durchführt.

#### logistische Regression </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) </br>
#### k-Neares Neighbour </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/kNearest_Neighbor_fuer_Bildklassifizierung) </br>
#### Random Forest </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Random_Forest_fuer_Bildklassifizierung)



<br>


# Weitere externe Informationen/Quellen
[Installieren von Bibliotheken mittels requirement.txt](https://note.nkmk.me/en/python-pip-install-requirements/) <br>

[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)<br>

[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)<br>

[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html) 
<br>

[SVM Model Implementierungs Guide](https://rpubs.com/Sharon_1684/454441)
<br>

[SVM Dokumentation](https://scikit-learn.org/stable/modules/svm.html)
