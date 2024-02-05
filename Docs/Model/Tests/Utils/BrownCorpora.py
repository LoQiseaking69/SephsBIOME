import nltk
import os
import datetime

# Get the current timestamp
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# Create a unique folder name with the timestamp in the current working directory
unique_folder = f"brown_corpus_{timestamp}"
custom_folder = os.path.join(os.getcwd(), unique_folder)

# Create the unique folder
if not os.path.exists(custom_folder):
    os.makedirs(custom_folder)

# Set the NLTK data path to include your custom folder
nltk.data.path.append(custom_folder)

# Download the Brown Corpus to the unique folder
nltk.download('brown', download_dir=custom_folder)