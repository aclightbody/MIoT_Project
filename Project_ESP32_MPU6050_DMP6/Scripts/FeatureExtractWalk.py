# Create venv: https://www.youtube.com/watch?v=O0bYaxUINnE
''' 
To do pip install: Need to install virtual environtment in in command prompt of VS Code 
(not powershell)  "python -m venv .venv", then 
activate venv (virtual environment) with ".venv\Scripts\activate"   
https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/
'''
import csv
import pandas

with open('..\\Data\\FeatureExtractWalkData.txt', newline='') as file:
    # for line in filestream:
    #     currentline = line.split(",")
    data = list(csv.reader(file))

row = 0
output = [["Label","axyStdDevL","axyStdDevR"]]

''' 
Extract the axyStDevL and axyStdDevR that correspond to the max axyStdDevR
for each sample period and insert 1 at the start of each rowto indicate
classification label
'''
for i in data:
    if (i[0]=='---------------'):
        row += 1
        output.append([-1,0,0])
        
    if (i[0].isnumeric() and len(i)==6):
        if (int(i[5]) > int(output[row][2])):
            output[row][1] = int(i[4])
            output[row][2] = int(i[5])

# with open('..\\Data\\myfile.csv','w') as myfile:
#     wr = csv.writer(myfile)
#     wr.writerow(output)
    
# with open('your_file.txt', 'w') as f:
#     for line in output:
#         f.write(f"{line}\n")
        
pandas.DataFrame(output).to_csv("..\\Data\\FeatureExtractWalk.csv",header=False,index=False)