# Direkter Vergleich aller Modelle 

Das vorliegende GitHub Repo kombiniert die bereits besprochenen vier Klassifizierungs-Use-Cases mit den Modellen logistische Regression, SVM, kNN und Random Forest. Es wird ein wieder versucht einen Hammer in dem Arbeitsbereich zu erkennen. 


## Einzelne Use-Cases und Modelle
| Model | Storyboard link | GitHub link | sklearn Dokumentation | 
|---|---|---|---|
| logistische Regression | [Storyboard](http://www.aiav.technikum-wien.at/) | [GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Logistische_Regression_fuer_Bildklassifizierung) | [Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LogisticRegression.html) |  
| Support Vector Machine | [Storyboard](http://www.aiav.technikum-wien.at/) | [GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Support_Vector_Machine_fuer_Bildklassifizierung) | [Dokumentation](https://scikit-learn.org/stable/modules/svm.html) |
| k-Neares Neighbour | [Storyboard](http://www.aiav.technikum-wien.at/) | [GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/kNearest_Neighbor_fuer_Bildklassifizierung) | [Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KNeighborsClassifier.html) |
| Random Forest | [Storyboard](http://www.aiav.technikum-wien.at/) | [GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Random_Forest_fuer_Bildklassifizierung) | [Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.ensemble.RandomForestClassifier.html) | 

</br>

# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. Dies weicht nicht ab von den vorherigen Use-Cases dieser art. 

Diese drei Bibliotheken geben das Grundgerüst vor. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. Wurden die vier einzelnen Klassifizierungs-Use-Cases schon erfolgreich ausgeführt, muss hier allerdings nichts mehr nachinstalliert werden. 

# Ordnerstruktur
Es wird erneut ein Datensatz an Bildern benötigt um die Modelle zu trainieren. Hierzu wurde wieder ein [Tool_Data](./Tool_Data) Ordner angelegt. In dem Ordner können einfach die Fotos getauscht werden falls ein eigener Datensatz geladen werden soll. So können individuelle Datensätze auch trainiert werden. Die Unterordner sind in unserem Fall [Hammer](./Tool_Data/Hammer) und [Workspace](./Tool_Data/Workspace). Diese geben gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use-Case angepasst werden, so können einfach Ordnernamen und (wie angesprochen die) Bilder getauscht werden. 

Das Noteboook [Combination_classification](Combination_classification.ipynb) ist der Beispielcode für die Implementierung aller vier bereits uns bekannten Modelle.


# Ergebnisse
Das unten angeführte [GIF](./demo/Demo_1x.gif) zeigt ein Beispielverhalten des Use-Cases. Eine Webcam ist über dem Arbeitsbereich positioniert und klassifiziert den Kamerastream. Es wird im Bild ausgegeben ob das Model denk, dass ein Hammer im Arbeitsbereich ist oder nicht. Dies ist nichts neues im vergleich zu unseren vorherigen Use-Cases. Der Unterschied ist, dass dies 4 mal pro Frame durchgeführt wird und zwar für jedes Model. 






![Abbildung 1](demo/Demo_1x.gif)

</br>

# Weitere externe Informationen/Quellen

### Hilfreiche Guides
[Installieren von Bibliotheken mittels requirement.txt](https://note.nkmk.me/en/python-pip-install-requirements/) </br>
[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)</br>
[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)</br>
[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html)</br>
[Implementierungs Guide zu ML Modellen](https://rpubs.com/Sharon_1684/454441)