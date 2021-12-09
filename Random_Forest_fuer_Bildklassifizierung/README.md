# Random Forest 

Hier befindet sich der [Beispielcode](./miniUsecase15_RandomForest.ipynb) für das Implementieren einer Support Vector Machine. Der Use-Case befasst sich mit der Klassifizierung von Bildern. Genauer gesagt, wollen wir herausfinden ob in einem Bild – beziehungsweise einem Kamerastream – sich ein Hammer befindet oder nicht. Dies ist auch der 4. Use-Case dieser Reihe. Die angesprochene Problemstellung wurde schon mit drei weiteren Modellen in den Use-Cases [Support Vector Machine](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_12_SVM), [Logistische Regression](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_11_logistic_reg) und [k-Nearest Neighbor](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/miniUsecase_15_Random_Forest) behandelt. 
Die Theorie zu dem Random Forest Model ist im [Storyboard](15_Storyboard_Random_Forest.pdf) zu finden. 

# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/stable/modules/svm.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. Wie das genau funktioniert ist auch in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) beschrieben.

# Ordnerstruktur
 Das Random Forest Model benötigt - wie auch die anderen drei Modelle - zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze Trainiert werden. Im Ordner [Tool_Data](./Tool_Data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo/Random_Forest_Test.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild direkt eingeblendet, ob das Model einen Hammer im Frame erkannt hat oder nicht. 

![Abbildung 1](demo/Random_Forest_Test.gif)

# Was nun? 
In dem Use-Case haben wir uns mit der Klassifizierung von Bildern mittels dem Random Forest Model befasst. Dies war der letzte der vier Klassifizierungen mit "klassischen" Modellen die für eine Klassifizierung eingesetzt werden können. Es gibt natürlich noch weitere Modelle und auch deutlich komplexere Modelle. Die drei vorherigen Modelle sind hier nocheinmal verlinkt. 

#### logistische Regression </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) </br>
### Support Vector Machine </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Support_Vector_Machine_fuer_Bildklassifizierung) </br>
#### k-Neares Neighbour </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/kNearest_Neighbor_fuer_Bildklassifizierung) </br>



Ebenso wird im Theorieteil immer von einem CNN gesprochen und dass dieses CNN vermutlich bessere Ergebnisse liefern kann. Was das genau ist und wie so ein CNN Funktioniert wird hier beschrieben. (coming soon)



# Weitere externe Informationen/Quellen
[Installieren von Bibliotheken mittels requirement.txt](https://note.nkmk.me/en/python-pip-install-requirements/) </br>
[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)</br>
[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)</br>
[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html) </br>
[Random Forest Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.ensemble.RandomForestClassifier.html)</br>
[Implementierungs Guide zu ML Modellen](https://rpubs.com/Sharon_1684/454441)</br>