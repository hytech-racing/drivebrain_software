import cantools

# Load DBC file
db = cantools.database.load_file('hytech_179.dbc')

# Hardcoded CAN frame (ID and hex payload from your log)
can_id = 0x40D

# 40D#00000000006147AF
data = bytes.fromhex('00000000006147AF')  # Example from your log

# Decode and print the result
decoded = db.decode_message(can_id, data)
print(f"Decoded 0x{can_id:X}: {decoded}")
