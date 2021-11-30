# Logistische Regression

Hier befindet sich der [Beispielcode](./miniUsecase11_logistic_regression.ipynb) für das Implementieren einer Logistischen Regression. Der Use-Case befasst sich mit der Klassifizierung von Bildern. Genauer gesagt, wollen wir herausfinden ob in einem Bild – beziehungsweise einem Kamerastream – sich ein Hammer befindet oder nicht. 

# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LogisticRegression.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. 

# Ordnerstruktur
 Die Logistische Regression benötigt zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze Trainiert werden. Im Ordner [Tool_Data](./Tool_Data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo/webcam_demo.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild ausgegeben ob das Model denk, dass ein Hammer im Arbeitsbereich ist oder nicht. 

![Abbildung 1](demo/webcam_demo.gif)

# Was nun? 
In dem Use-Case haben wir uns mit der Klassifizierung von Bildern mittels der Logistischen Regression befasst. Wenn Sie weiteres Interesse an klassifizierungs Modellen haben, empfehlen wir folgende Use-Cases auf der Plattform. All diese drei weiteren Use-Cases sind nach demselben Schema aufgebaut. Sie haben allerdings jeweils ein anderes Model, welches die Klassifizierung durchführt.

### Support Vector Machine </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Support_Vector_Machine_fuer_Bildklassifizierung) </br>
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
