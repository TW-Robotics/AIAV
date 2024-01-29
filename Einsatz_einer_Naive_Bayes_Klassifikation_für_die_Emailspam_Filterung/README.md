# Einsatz einer Naive Bayes Klassifikation für die Emailspam Filterung

Jede Person mit einem Emailaccount wird bereits Erfahrungen mit Spam gemacht haben. Entweder man soll einen hilflosen Prinzen aus der finanziellen Krise helfen oder ist nur einen Klick davon entfernt ein neues iPhone zu gewinnen. Die Ideen hinter Spamnachrichten sind endlos und können nach und nach richtig nervig werden, wenn sie öfters erscheinen.

Eine Abhilfe ist dabei der Spamfilter, welcher hinterlistigen Nachrichten entdeckt und diese anschließend in den Spamordner verbannt. Doch wie genau entscheidet der Filter eigentlich, welche Nachrichten Spam sind und welche nicht? Genau diese Antwort soll dieses Notebook liefern, in der ein Klassifizierungsalgorithmus, genauer gesagt ein multinominal Naive Bayes Klassifizierer, eingesetzt wird, um Spam Emails zu detektierten.

Der Naive Bayes beruht auf dem Bayes Theorem, welcher Wahrscheinlichkeiten bezogen auf Ereignisse berechnet. Für unsere konkretes Beispiel des Filters, wird für jede Email die Anzahl an Wörtern gezählt, um anschließend bestimmen zu können, ob es sich tatsächlich um Spam handelt, oder doch nur um eine ganz normale Email. Dabei geht der Klassifizier davon aus, dass die einzelnen Variablen unabhängig sind.

Eine mögliche Implementierung des Klassifizierers kann wie in diesem Projekt gezeigt, umgesetzt werden. Dabei wurde der Datensatz "spam_ham_dataset" verwendet, welcher eine große Anzahl an englischen Emails und Spamemails beinhaltet. Von den insgesamt 5170 Emails sind 1500 mit Spam klassifiziert.
Als Grundlage wurde die [sklearn](https://scikit-learn.org/stable/) Bibliothek verwendet.

## Ausführung

Zum Ausführen des Projektes die Datei start.bash im Terminal ausführen. Nach der Installierung in der virtuellen Umgebung, wird der Naive Bayes Klassifizierer am Datensatz [spam_ham_dataset](https://www.kaggle.com/ayhampar/spam-ham-dataset/data).

Alternativ kann auch das vorhandene Jupyter Notebook ausgeführt werden, wo zusätzliche Beschreibungen zum Code vorhanden sind.

## Zusätzliche Informationen

Für mehr Informationen kann das [AIAV Naive Bayes Video](https://youtu.be/ioDdAE6AOMQ) betrachtet werden, in der die mathematischen Hintergründe erklärt werden.
Zusätzlich finden sich viele Informationen und Beispiele zum Thema Machine Learning und Künstlicher Intelligenz auf unserer offiziellen [AIAV Website](https://www.aiav.technikum-wien.at/).


<p align="center">
  <img src="img/wien_ma23.png" width="480"> <img src="img/FH_Technikum_Wien_logo.png" width="260">
</p>
