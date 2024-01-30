# Von_Large_Language_Models_zum_Vision_Transformer

## Der Use Case

In diesem Use Case wird die grundlegende Funktionsweise eines Vision Transformers erklaert und auf eine Klassifikationsaufgabe, traniert und angewandt. Der Code im Jupyterlab-notebook wird analog zu dem Vision Transformer Paper [An Image is Worth 16x16 Words: Transformer for Image Recognition at Scale](https://arxiv.org/pdf/2010.11929.pdf) erstellt und auf einen neuen Datensatz getestet. Der Datensatz hierfuer ist der [WOD: Web Object Dataset](https://www.acin.tuwien.ac.at/vision-for-robotics/software-tools/autonomous-robot-indoor-dataset/). Das Jupyterlab-notebook ist eine ueberarbeitete Form von [GitHub Repository von mrdbourke](https://github.com/mrdbourke/pytorch-deep-learning), welcher ein frei zugaengliches PyTorch Tutorial auf seiner [Homepage](https://www.learnpytorch.io/) bereitstellt.


## Ergebnisse
In diesem Abschnitt werden die Ergebnisse dieses Use-Case's vorgestellt. Einerseits ist die theoretische Fundierung entsprechend Abbildung 1 und der Gleichung 1  ein wichtiger Bestandteil dieses Use-Cases. Weiters wurden ein von Null trainiertes Vision Transformer Modell mit den selbem (limitierten) Datensatz trainiert wie ein vortrainiertes. Die Ergebnisse hierfuer seehen sie in den folgenden zwei Abbildungen mit den entsprechenden Accuracy Kurven fuer Test und Trainings-Datensaetze:



## Diskussion 

Es hat sich gezeigt, dass der Vision Transformer erst nach Anwendung einer bereits vortrainierten Version zielfuehrende Ergebnisse liefert. Dies ist sinnvoll, da Vision Transformer im Allgemeinen datenintensive Modelle sind und keine starke induktive Verzerrung in das Modell einbringen. Es ist daher nicht praktikabel, Vision Transformer von Grund auf zu trainieren, da dies einerseits einen riesigen Datensatz und andererseits enorme Rechenkapazitaeten erfordert. Wenn man jedoch ein bereits trainiertes Modell verwendet, kann man, wie wir gesehen haben, schnell gute Ergebnisse erzielen. Beim derzeitigen \href{https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9716741}{Stand der Forschung} kann man sagen, dass CNN's bei kleinen Datensaetzen gut abschneiden und Transformer bei grossen Datensaetzen besser.
