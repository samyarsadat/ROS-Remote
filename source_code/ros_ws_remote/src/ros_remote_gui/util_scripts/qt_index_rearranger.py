# This is a small script for making the numbering of horizontalSpacers, verticalSpacers
# horizontalLayouts, verticalLayouts in QT '.ui' files sequential.
# This has no operational impact on the program or UI, and only exists because
# non-sequential numbering annoys me.
#
# Written by Samyar Sadat Akhavi, 2024.


import sys

fix_name_array = ["horizontalSpacer", "verticalSpacer", "horizontalLayout", "verticalLayout", "gridLayout"]
filename = sys.argv[1]
print(f"Fixing indexes for: {filename}")

for name in fix_name_array:
    print(f"Fixing name: {name}")
    filestr = ""

    with open(filename, mode="r") as file:
        replace_index = 1

        for line_no, line in enumerate(file):
            start_index = line.find(name)
            end_index = 0

            if start_index != -1:
                for index, char in enumerate(line[start_index:]):
                    if char == '"':
                        end_index = start_index + index

                if not end_index > start_index:
                    print(f"Error! String end not found! (Line: {line_no + 1})")
                    file.close()
                    exit(1)

                replacement = f"{name}_{replace_index}"
                print(f"Fixing index for: {line[start_index:end_index]}, replaced with: {replacement}")
                line = line.replace(line[start_index:end_index], replacement)
                replace_index += 1
            filestr += line
        file.close()

    with open(filename, mode="w") as file:
        file.write(filestr)
        file.close()