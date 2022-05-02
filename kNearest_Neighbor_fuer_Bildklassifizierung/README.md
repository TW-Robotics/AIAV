# k-Nearest Neighbor 

Hier befindet sich der [Beispielcode](./miniUsecase15_RandomForest.ipynb) für das Implementieren einer k-Nearest Neighbor Klassifizierung. Der Use Case befasst sich mit der Klassifizierung von Bildern. 

Genauer gesagt wollen wir handgeschriebene Buchstaben klassifizieren. Die Theorie des kNN Models ist im beiliegenden [Storyboard]() beschrieben.

# Bibliotheken
Die Implementierung ist erneut mittels der [scikit-learn](https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KNeighborsClassifier.html) Bibliothek in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Alle benötigten Bibliotheken sind in der [requirements-Datei](./requirements.txt) aufgelistet und können mithilfe dieser installiert werden. Wie das genau funktioniert ist in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) beschrieben. 

# Ordnerstruktur
 Das kNN Model benötigt - wie auch die anderen Modelle - zum Trainieren einen Datensatz an Bildern. Der Beispielcode ist so aufgebaut, dass innerhalb der vorgegebenen Ordnerstruktur einfach die Fotos getauscht werden können. So können individuelle Datensätze Trainiert werden. Im Ordner [data](./data) befinden die Unterordner, die gleichzeitig auch die Klassen vorgeben. Diese sind in unserem Fall die Buchstaben 'a' bis 'z'. Soll der Code für einen anderen Use Case angepasst werden, so können einfach Ordnernamen und Bilder getauscht werden. 


# Ergebnisse
Das unten angeführte [GIF](./demo.gif) zeigt ein Beispielverhalten des Use Cases. Es werden dem Modell Bilder übergeben. Es versucht dann die Klasse herauszufinden und blendet das Ergebnis ein. 

![Abbildung 1](demo.gif)


Das Model hat eine Accuracy (Genauigkeit) von rund 84%. Ein typisches Storyboard auf der AIAV Webseite hat rund 10000 Zeichen (ohne Leerzeichen). Nehmen wir an, die Storyboards bestehen nur aus Buchstaben und sind handschriftlich geschrieben. Das bedeutet, dass wir mit dem Modell ca. 1600 Fehler machen würden, wenn wir den Text scannen würden. Das ist für eine Praxisanwendung wahrscheinlich nicht ausreichend. 


# Was nun? 
In diesem Use Case haben wir uns mit der Klassifizierung von Bildern mittels k-Nearest Neighbor befasst. Wenn Sie weiteres Interesse an Klassifizierungmodellen haben, empfehlen wir folgende Use Cases auf der Plattform. 

#### logistische Regression </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/main/Logistische_Regression_fuer_Bildklassifizierung) </br>
### Support Vector Machine </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/main/Support_Vector_Machine_fuer_Bildklassifizierung) </br>
#### Random Forest </br>
[Storyboard](http://www.aiav.technikum-wien.at/) </br>
[GitHub](https://github.com/TW-Robotics/AIAV/tree/main/Random_Forest_fuer_Bildklassifizierung)

Ebenso haben wir angesprochen, dass in der Praxis klassische Methoden nicht immer ausreichend sind. Um dieses Problem zu lösen, kann auf ein komplexeres Modell zurückgegriffen werden. Ein Beispiel für so ein Modell sind Convolutional Neural Networks (CNN).
[Coming Soon]

<br>


# Weitere externe Informationen/Quellen
[Installieren von Bibliotheken mittels requirements.txt](https://note.nkmk.me/en/python-pip-install-requirements/) <br>

[Erstellen eines Dictionary für die verarbeitung der Trainingsbilder](https://kapernikov.com/tutorial-image-classification-with-scikit-learn/)<br>

[Implementierung einer PCA](https://medium.com/@sebastiannorena/pca-principal-components-analysis-applied-to-images-of-faces-d2fc2c083371)<br>

[Visualisierung einer PCA](https://jakevdp.github.io/PythonDataScienceHandbook/05.02-introducing-scikit-learn.html) 
<br>

[kNN Model Implementierungsguide](https://rpubs.com/Sharon_1684/454441)<br>

[kNN Dokumentation](https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.KNeighborsClassifier.html)