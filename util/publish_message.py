import requests
import argparse

def main():
    parser = argparse.ArgumentParser(description='visualize data')
    parser.add_argument('-i', '--input', required=True, help='Input data path for visualization')
    args = parser.parse_args()

    url  = 'http://localhost:1234/v1/publish-message'
    data = {'name': args.input}

    x = requests.post(url, json=data)
    print(x)

if __name__ == "__main__":
    main()
