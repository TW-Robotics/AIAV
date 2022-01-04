# Einsatz einer Naive Bayes Klassifikation für die Emailspam Filterung

Jede Person mit einem Emailaccount wird bereits Erfahrungen mit Spam gemacht haben. Entweder man soll einen hilfslosen Prinzen aus der finanziellen Krise helfen oder ist nur einen Klick davon entfernt ein neues IPhone zu gewinnen. Die Ideen hinter Spamnachrichten sind endlos und können nach und nach richtig nervig werden, wenn sie öfters erscheinen.

Eine Abhilfe ist dabei der Spamfilter, welcher hinterlistigen Nachrichten entdeckt und diese anschließend in den Spamordner verbannt. Doch wie genau entscheidet der Filter eigentlich welche Nachrichten Spam sind und welche nicht? Genau diese Antwort soll dieses Notebook liefern, in der ein Klassifierungsalgorithmus, genauer gesagt ein multinominal Naive Bayes Klassifizierer, eingesetzt wird, um Spam emails zu detektierten.

Der Naive Bayes beruht auf dem Bayes Theorem, welcher Wahrscheinlichkeiten bezogen auf Ereignisse berechnet. Für unsere konkretes Beispiel des Filters, wird für jede Email die Anzahl an Wörtern gezählt, um anschließend bestimmen zu können, ob es sich tatsächlich um Spam handelt, oder doch nur um eine ganz normale Email. Dabei geht der Klassifizier davon aus, dass die einzelnen Variablen unabhängig sind.

Für die Berechnung der Wahrscheinlichkeiten werden durch folgende Gleichung berechnet:

𝑃(𝑆𝑝𝑎𝑚|𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡)=𝑃(𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡|𝑆𝑝𝑎𝑚)⋅𝑃(𝑆𝑝𝑎𝑚)𝑃(𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡)=𝑃(𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡|𝑆𝑝𝑎𝑚)⋅𝑃(𝑆𝑝𝑎𝑚)𝑃(𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡|𝑆𝑝𝑎𝑚)⋅𝑃(𝑆𝑝𝑎𝑚)+𝑃(𝑆𝑢𝑐ℎ𝑤𝑜𝑟𝑡|𝑘𝑒𝑖𝑛𝑆𝑝𝑎𝑚)⋅𝑃(𝑘𝑒𝑖𝑛𝑆𝑝𝑎𝑚)

Die Wahrscheinlichkeit, dass eine Email Spam ist unter der Bedingung des Wortes P(Spam|Suchwort) ist gesucht. Über das Bayes Theorem kann dies wie auf der rechten Seite der Gleichung dargestellt berechnet werden und zwar über die Wahrscheinlichkeit, dass ein Wort in einer Spamemail vorkommt P(Suchwort|Spam), multipliziert mit der Wahrscheinlichkeit dass es sich um eine Spamemail handelt P(Spam). Diese Wahrscheinlichkeit muss noch durch die Wahrscheinlichkeit P(Suchwort) dividiert werden, welche eine Addition der Wahrscheinlichkeit des Wortes in Spamemails und normalen Emails sind.

Eine mögliche Implementierung des Klassifizieres kann wie folgt umgesetzt werden. Dabei wurde der Datensatz "spam_ham_dataset" verwendet, welcher eine große Anzahl an englischen Emails und Spamemails beinhaltet. Von den ingesamt 5170 Email sind 1500 mit Spam klassifiziert.
