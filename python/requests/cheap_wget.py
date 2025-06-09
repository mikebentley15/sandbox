import requests
import sys


def download_to_string(url):
    try:
        response = requests.get(url, timeout=5, verify=True)
        response.raise_for_status()  # Raise exception for HTTP errors
        return response.text
    except requests.exceptions.RequestException as e:
        print(f"Error downloading content: {e}")
        return None


# Example usage
url = sys.argv[1]
content = download_to_string(url)
if content:
    print(f"Downloaded {len(content)} characters")
    print(content)
