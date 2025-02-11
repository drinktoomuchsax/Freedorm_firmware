import csv
import random
import string
from datetime import datetime

def generate_serial_number(seed, count):
    random.seed(seed)  # 使用用户提供的种子
    serial_numbers = set()  # 用集合确保不重复

    while len(serial_numbers) < count:
        # 生成两个大写字母
        letters = ''.join(random.choices(string.ascii_uppercase, k=2))
        # 生成两个数字
        numbers = ''.join(random.choices(string.digits, k=2))
        # 生成完整的序列号
        serial_number = letters + numbers
        serial_numbers.add(serial_number)  # 如果重复，集合不会添加

    return list(serial_numbers)

def create_serial_table(serial_numbers):
    # 当前时间
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # 用户旺旺号（假设你会根据实际情况填写）
    user_wangwang = "用户旺旺号"  # 需要替换成实际值或者让用户输入

    table = []
    for serial_number in serial_numbers:
        table.append([
            serial_number,  # 序列号
            "否",  # 是否使用
            "1.0",  # 对应版本
            current_time,  # 售出时间
            user_wangwang  # 用户旺旺号
        ])
    
    return table

def save_to_csv(table, filename="serial_numbers.csv"):
    # 保存为CSV文件
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        # 写入表头
        writer.writerow(["序列号", "是否使用", "对应版本", "售出时间", "用户旺旺号"])
        # 写入数据行
        writer.writerows(table)

if __name__ == "__main__":
    seed = input("请输入种子: ")
    count = int(input("请输入需要生成的序列号数量: "))
    
    # 生成序列号
    serial_numbers = generate_serial_number(seed, count)
    
    # 创建序列号表格
    table = create_serial_table(serial_numbers)
    
    # 保存为CSV
    save_to_csv(table)

    print("CSV文件已生成。")
