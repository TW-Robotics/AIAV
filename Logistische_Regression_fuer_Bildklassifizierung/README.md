# Logistische Regression

Hier befindet sich der [Beispielcode](./miniUsecase11_logistic_regression.ipynb) für das Implementieren einer logistischen Regression. Der Use-Case befasst sich mit der Klassifizierung von Bildern. Genauer gesagt, wollen wir herausfinden ob sich in einem Bild – beziehungsweise einem Kamerastream – ein Hammer befindet oder nicht. Die Theorie und der Aufbau ist im [Storyboard](11_Storyboard_logistic_regression.pdf) genau beschrieben. 

# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LogisticRegression.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. 

# Ordnerstruktur
Die logistische Regression benötigt zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze trainiert werden. Im Ordner [Tool_Data](./Tool_Data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 

Das Noteboook [miniUsecase11_logistic_regression](miniUsecase11_logistic_regression.ipynb) bietet eine Implementation der logistischen Regression. Falls weiteres Interesse besteht, kann das Notebook [miniUsecase11_logistic_regression_with_visuals](miniUsecase11_logistic_regression_with_visuals.ipynb) ausgeführt werden. Hier werden weitere Informationen wie eine Visualisierung der PCA ausgegeben. 


# Ergebnisse
Das unten angeführte [GIF](./demo/webcam_demo.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild ausgegeben ob das Model denk, dass ein Hammer im Arbeitsbereich ist oder nicht. 

![Abbildung 1](demo/webcam_demo.gif)

# Was nun?
In dem Use-Case haben wir uns mit der Klassifizierung von Bildern mittels der logistischen Regression befasst. Wenn Sie weiteres Interesse an klassifizierungs Modellen haben, empfehlen wir auch folgende Use-Cases auf der AIAV-Platform. All diese drei weiteren Use-Cases sind nach demselben Schema aufgebaut, allerdings jeweils mit eim anderen Model, welches die Klassifizierung durchführt: 

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
