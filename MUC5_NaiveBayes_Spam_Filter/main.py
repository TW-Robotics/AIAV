import sys
from NB_Classifier import NB_Class

def main():
    print("Start Program.")
    inputData = sys.argv[1]
    nlp = NB_Class()
    nlp.pipeline(inputData)

if __name__ == "__main__":
    main()
