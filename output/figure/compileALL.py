import os; import glob

names = glob.glob('./*.tex')
# for el in names: os.system("lualatex -shell-escape " + el)

os.system("rm *.log")
os.system("rm *.aux")
