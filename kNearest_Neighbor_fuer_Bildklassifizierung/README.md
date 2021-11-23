# k-Neares Neighbor 

Hier befindet sich der [Beispielcode](./miniUsecase15_RandomForest.ipynb) für das Implementieren einer Support Vector Machine. Der Use-Case befasst sich mit der Klassifizierung von Bildern. Genauer gesagt, wollen wir herausfinden ob in einem Bild – beziehungsweise einem Kamerastream – sich ein Hammer befindet oder nicht. Das ist der dritte Use-Case dieser Reihe. Die beiden vorherigen Use-Cases haben sich mit der selben Problemstellung befasst allerdings mit dem einsatz einer [logistischen Regression](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) beziehungsweise einer [Support Vector Machine](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_12_SVM). 

# Bibliotheken
Die Implementierung ist erneut mittels der [scikit-learn](https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KNeighborsClassifier.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. Wie das genau funktioniert ist in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) beschrieben. 

# Ordnerstruktur
 Das kNN Model benötigt - wie auch die anderen Modelle - zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze Trainiert werden. Im Ordner [Tool_Data](./Tool_Data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo/kNN_Test.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild eingeblendet ob das kNN-Model in dem Frame einen Hammer erkannt hat oder nicht.

![Abbildung 1](demo/kNN_Test.gif)


# Was nun? 
In dem Use-Case haben wir uns mit der Klassifizierung von Bildern mittels der Support Vector Machine befasst. Wenn Sie weiteres Interesse an klassifizierungs Modellen haben, empfehlen wir folgende Use-Cases auf der Plattform. All diese drei weiteren Use-Cases sind nach demselben Schema aufgebaut. Sie haben allerdings jeweils ein anderes Model, welches die Klassifizierung durchführt.

Use-Case 11: [Logistische Regression für Klassifizierungsprobleme](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_12_SVM) <br>
Use-Case 13: [Support Vector Machine Klassifizierung](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_11_logistic_reg) <br>
Use-Case 15: [Random Forest Klassifizierung](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_15_Random_Forest)

<br>


# Weitere externe Informationen/Quellen
[Installieren von Bibliotheken mittels requirement.txt](https://note.nkmk.me/en/python-pip-install-requirements/) <br>

[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)<br>

[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)<br>

[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html) 
<br>

[kNN Model Implementierungs Guide](https://rpubs.com/Sharon_1684/454441)<br>

[kNN Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KNeighborsClassifier.html)