# Convolutional Neural Network für Detektion

Das vorliegende Github Repo zeigt die implementierung zu dem Use-Case der CNN Detektion. Wir werden ein CNN implementieren mit dem wir einen mobilen Roboter detektieren wollen. Die Theorie und das Storyboard ist [hier](https://www.aiav.technikum-wien.at/ai-anwenden) zu finden.  

![Output Example](/output/output.png)


# Tensorflow EfficientDet-Lite2 Test Platform
https://tfhub.dev/tensorflow/efficientdet/lite2/detection/1


## Bibliotheken

Das Notebook benötigt, wie auch bei den anderen Use-Cases, gewisse Bibliotheken. Diese können über die [requirements-Datei](./requirements.txt) installiert werden. Wie das genau funktioniert ist in diesem [Tutorial](https://note.nkmk.me/en/python-pip-install-requirements/) auch beschrieben.  

Die Implementierung des [CNN Models](https://www.tensorflow.org/lite/tutorials/model_maker_object_detection) ist mittels [Tensorflow2](https://www.tensorflow.org/) in der Programmiersprache [Python](https://docs.python.org/3/) umgesetzt. Für die Einbindung der Kamera und Bildverarbeitung wird [OpenCV](https://opencv.org/) verwendet. 

Aufgrund von unterschiedlichen Strukturen von Tensorflow ist es wichtig, dass die richtige Tensorflow Version installiert ist. Die notwendige Version ist in der [requirements-Datei](./requirements.txt) zu finden.


## Eigener Datensatz und Annotieren

Um unser Modell zu trainieren benötigen wir einen Datensatz. Wir wollen in unserem Use-Case den MiR Roboter tracken. Das heißt, dass wir einen Datensatz benötigen, woraus das CNN lernen kann wie den dieser aussieht. 

Dazu nehmen wir viele Bilder vom MiR auf. Da wir nicht nur wissen wollen ob auf unserem Bild ein MiR zu sehen (Stichwort Use-Case [Klassifizierung mittels CNN](https://www.aiav.technikum-wien.at/) ist sondern auch wissen wollen, wo dieser ist müssen wir die Bilder annotieren.  

Annotieren ist die Art dem CNN zu sagen, wo den unser Objekt sich in dem Bild befindet. Für unser Modell benötigen wir das sogenannte Pascal Voc XML Format. Sprich zu jedem Bild haben wir eine .xml Datei die uns beschreibt wo sich unser Objekt befindet. 

Da wir nicht die ersten sind, die sich mit dem Problem beschäftigen gibt es Annotation-Tools. Die helfen uns diese Beschreibung zu erstellen. Für den Use-Case haben wir die Webseite [cvat](https://cvat.org/) verwendet. Der große Vorteil von solchen Tools ist, dass man meist die Annotierung für verschiedene Modelle exportieren kann. So können wir auch unsere Informationen einfach als XML Datei exportieren. 


## Ordnerstruktur

Damit unser Notebook richtig ausgeführt werden kann, benötigen wir eine gewisse Ordner Struktur. 

In dem Ordner [Dataset](./dataset/) sind drei weitere Unterordner. Die beinhalten unsere Bilder und xml-Datein. Wie die Ordnernamen auch sagen ist der Test-Ordner für Testbilder, der Validate-Ordner für das Validieren des Modells und der Train-Ordner beinhaltet die Trainingsbilder. Zu jedem Bild muss wie schon angesprochen auch die passende xml-Datei dazu abgelegt werden. 

Weiters gibt es noch die requirements-Datei welche alle notwendigen Bibliotheken beinhaltet, sowie das Notebook selbst. 


- Dataset
    - Test
    - Train
    - Validate
- CNN_detection_UseCase.ipynb
- Readme
- requirements



## Ergebnisse
![MiR Tracker](output/MiR_Tracker_2.gif)


## Was nun? 


## Weitere externe Informationen/ Quellen
