# Einsatz einer Naive Bayes Klassifikation für die Emailspam Filterung

Jede Person mit einem Emailaccount wird bereits Erfahrungen mit Spam gemacht haben. Entweder man soll einen hilfslosen Prinzen aus der finanziellen Krise helfen oder ist nur einen Klick davon entfernt ein neues IPhone zu gewinnen. Die Ideen hinter Spamnachrichten sind endlos und können nach und nach richtig nervig werden, wenn sie öfters erscheinen.

Eine Abhilfe ist dabei der Spamfilter, welcher hinterlistigen Nachrichten entdeckt und diese anschließend in den Spamordner verbannt. Doch wie genau entscheidet der Filter eigentlich welche Nachrichten Spam sind und welche nicht? Genau diese Antwort soll dieses Notebook liefern, in der ein Klassifierungsalgorithmus, genauer gesagt ein multinominal Naive Bayes Klassifizierer, eingesetzt wird, um Spam emails zu detektierten.

Der Naive Bayes beruht auf dem Bayes Theorem, welcher Wahrscheinlichkeiten bezogen auf Ereignisse berechnet. Für unsere konkretes Beispiel des Filters, wird für jede Email die Anzahl an Wörtern gezählt, um anschließend bestimmen zu können, ob es sich tatsächlich um Spam handelt, oder doch nur um eine ganz normale Email. Dabei geht der Klassifizier davon aus, dass die einzelnen Variablen unabhängig sind.

Eine mögliche Implementierung des Klassifizieres kann wie folgt umgesetzt werden. Dabei wurde der Datensatz "spam_ham_dataset" verwendet, welcher eine große Anzahl an englischen Emails und Spamemails beinhaltet. Von den ingesamt 5170 Email sind 1500 mit Spam klassifiziert.

## Ausführung

Zum ausführen des Projektes die Datei start.bash im Terminal ausführen. Nach der Installierung in der virtuellen Umgebung, wird der Naive Bayes Klassifizierer am Datensatz [spam_ham_dataset](https://www.kaggle.com/ayhampar/spam-ham-dataset/data).

Alternativ kann auch das vorhande Jupyter Notebook ausgeführt werden, wo zusätzliche Beschreibungen zum Code vorhanden sind.

## Zusätzliche Informationen

Für mehr Informationen kann das [AIAV Naive Bayes Video](https://youtu.be/ioDdAE6AOMQ) betrachtet werden, in der die mathematischen Hintergründe erklärt werden.
Zusätzlich finden sich viele Informationen und Beispeile zum Thema Machine Learning und Künstlicher Intelligenz auf unserer offziellen AIAV Website(https://www.aiav.technikum-wien.at/).


![alt text](https://github.com/TW-Robotics/AIAV/edit/devel_muster/MUC5_NaiveBayes_Spam_Filter/img/image.jpg?raw=true)
