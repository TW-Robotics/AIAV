from sklearn.feature_extraction.text import CountVectorizer
from sklearn.feature_extraction.text import TfidfTransformer
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split

from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score

from CSV_Converter import csvReader

class NB_Class:
    def __init__(self):
        self.count_vector = CountVectorizer()
        self.tfidf_transformer = TfidfTransformer()
        self.X = []
        self.y = []

    def pipeline(self,inputData):
        self.makeData(inputData)
        self.trainModel()

    def makeData(self,inputData):
        self.reader = csvReader(inputData)
        self.X, self.y = self.reader.readCSV()

    def trainModel(self):
        self.X_train_counts = self.count_vector.fit_transform(self.X)
        self.X_train_tfidf = self.tfidf_transformer.fit_transform(self.X_train_counts)

        x_train, x_test , y_train, y_test = train_test_split(self.X_train_tfidf,self.y, test_size = 0.1, shuffle = True)
        self.clf = MultinomialNB().fit(x_train, y_train)
        predicted = self.clf.predict(x_test)
        print("Classification report:\n",classification_report(predicted, y_test))
        print("Confusion matrix:\n",confusion_matrix(predicted, y_test))
        print("Accuracy score:",accuracy_score(predicted, y_test)*100,"%")
