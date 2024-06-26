import csv

def add_prefix_to_csv(input_file, output_file):
    with open(input_file, mode='r', newline='') as infile:
        reader = csv.reader(infile)
        header = next(reader)  # Skip the header
        
        rows = []
        for idx, row in enumerate(reader):
            rows.append([f'ego_{idx}'] + row)
        
    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(['id'] + header)  # Write the new header
        writer.writerows(rows)

if __name__ == '__main__':
    input_csv_file = './csv/0620.csv'  # 입력 파일 이름
    output_csv_file = './csv/0620.csv'  # 출력 파일 이름
    add_prefix_to_csv(input_csv_file, output_csv_file)
