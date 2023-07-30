filename = 'tst.zip'
N = 10
# new_data = b'Hello World'
new_data = b'PK\x03\x04\x14\x00\x00\x00\x08\x00'

# 读取文件前 N 个字节
with open(filename, 'r+b') as file:
    orig_data = file.read(N)

print('Original bytes:', orig_data)

# 修改前 N 个字节为指定数据
new_bytes = bytearray(new_data)
orig_bytes = bytearray(orig_data)
for i in range(N):
    orig_byte = orig_bytes[i]
    new_byte = new_bytes[i]
    orig_bytes[i] = new_byte
    new_bytes[i] = orig_byte

# 输出被替换掉的原字节

# 将修改后的数据写回文件
with open(filename, 'r+b') as file:
    file.seek(0)
    file.write(orig_bytes)

    # 将原始数据后面的数据一字节一字节地复制到修改后的位置
    file.seek(N)
    while True:
        byte = file.read(1)
        if not byte:
            break
        file.seek(-1, 1)
        file.write(byte)

    # 截断文件，删除多余的字节
    file.truncate(N + file.tell())

print('Done.')
