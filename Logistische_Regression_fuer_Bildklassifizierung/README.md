# Logistische Regression

Hier befindet sich der [Beispielcode](./miniUsecase11_logistic_regression.ipynb) für das Implementieren einer logistischen Regression. Wie schon im Storyboard besprochen, wollen wir mit diesem Modell Bilder von Kleidungsstücken klassifizieren. 

# Bibliotheken
Die Implementierung ist mittels der [scikit-learn](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LogisticRegression.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. 

Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können auch über diese installiert werden. 

__Durch Probleme in der Onlineansicht kann es vorkommen, dass die Bilder im Notebook hier auf Github nicht angezeigt werden. Sollte dies der Fall sein, können Sie sich den [AIAV Ordner](https://github.com/TW-Robotics/AIAV/archive/refs/heads/main.zip) herunterladen und [Notebook.html](Notebook.html) lokal im Browser anzeigen lassen.__

# Ordnerstruktur
Die logistische Regression benötigt zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze trainiert werden. Im Ordner [data](./data) befinden sich zwei Unterordner. Diese sind in unserem Fall [Dress](./data/Dress) und [Pullover](./data/Pullover). Dies gibt gleich die beiden Klassen für die Klassifizierung vor. Soll der Code für einen anderen Use Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo.gif) zeigt ein Beispielverhalten des Use Cases. Nach dem Trainieren werden dem Modell neue Bilder von Kleidungsstücken gezeigt. Das Modell klassifiziert anschließend diese und gibt die Prediction aus. 

![Abbildung 1](demo.gif)

Die sogenannte Accuracy (Genauigkeit) sagt uns aus wie gut unser Modell klassifizieren kann. Mit dem Testdatensatz wurde eine Accuracy von 97% erreicht. Das ist für die Praxis oft nicht gut genug. Wenn dieses Modell beispielsweise bei einer Verpackungsstation eingesetzt wird, bei der 1 Millionen Produkte am Tag verpackt werden, dann ist mit 30 000 Fehlern zu rechnen. 

# Was nun?
In diesem Use Case haben wir uns mit der Klassifizierung von Bildern mittels der logistischen Regression befasst. Wenn Sie weiteres Interesse an klassifizierungsmodellen haben, empfehlen wir auch folgende Use Cases auf der AIAV Platform: 

### Support Vector Machine </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Support_Vector_Machine_fuer_Bildklassifizierung) </br>
#### k-Nearest Neighbour </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/kNearest_Neighbor_fuer_Bildklassifizierung) </br>
#### Random Forest </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/devel_abdank/Random_Forest_fuer_Bildklassifizierung)

Ebenso haben wir angesprochen, dass in der Praxis klassische Methoden nicht immer ausreichend sind. Um dieses Problem zu lösen, kann auf ein komplexeres Modell zurückgegriffen werden. Ein Beispiel für so ein Modell sind Convolutional Neural Networks (CNN).
[Coming Soon]

<br>

# Weitere externe Informationen/Quellen
[Installieren von Bibliotheken mittels requirement.txt](https://note.nkmk.me/en/python-pip-install-requirements/) <br>
[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)<br>
[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)<br>
[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html)
