require('luamodbus')
mb = luamodbus.rtu()

mb:open('/dev/RS485-1', 9600, 'N', 8, 1, 'H')

mb:connect()

mb:setslave(1)

-- set bit at address 0 to 'on'
mb:writebits(0, true)

coil = mb:readcoils(0)
log(coil)

os.sleep(2)

mb:writebits(0, false)

coil = mb:readcoils(0)
log(coil)

mb:close()