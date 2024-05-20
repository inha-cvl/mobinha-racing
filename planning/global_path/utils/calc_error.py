import pandas as pd

# Load the data from the CSV files
file1 = pd.read_csv('./ref_lines/kiapi1.csv', header=None, names=['x', 'y'])
file2 = pd.read_csv('./paths/PreRound1.csv', usecols=['x', 'y'])

errors = file2 - file1
print(errors)
errors['error_magnitude'] = errors.pow(2).sum(axis=1).pow(0.5)  # Euclidean distance for error magnitude

# Print the mean error
mean_error = errors['error_magnitude'].mean()
print(f"Mean error magnitude: {mean_error}")