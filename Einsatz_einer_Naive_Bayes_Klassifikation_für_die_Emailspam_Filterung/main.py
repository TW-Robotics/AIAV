import sys
from NB_Classifier import NB_Class

def main():
    print("\n \033[92m ******************* Start Naive Bayes Klassifizierung ******************* \033[0m \n")
    inputData = sys.argv[1]
    print("Datensatz: ", inputData, "\n")
    nlp = NB_Class()
    nlp.pipeline(inputData)

if __name__ == "__main__":
    main()
