# bme280

Python module to query the Bosch Sensortec BME280 temperature,
pressure and humidity sensor.

For details of the sensor, refer to the datasheet available here:

<https://www.bosch-sensortec.com/bst/products/all_products/bme280>

Features:

* Fast sensor access by using burst reads. This means the lib is
  quickly initialized.
  
* use polling to detect when sensor has finished a measurement (no
  hardcoded delay).

## Usage

You can import the bme280.py module into your application and
instanciate the BME280 class. See the end of the bme280.py file for a
full example of how to use the BME280 class. Here a short example:

```python
from smbus import SMBus

# open /dev/i2c-1
bus = SMBus(1)
sensor = BME280(bus, address=0x76)
res = sensor.sample(os=2)

print("temperature={0}".format(res.temperature))
print("pressure={0}".format(res.pressure))
print("humidity={0}".format(res.humidity))
```

Or you can run bme280.py as a stand-alone application. It will query
the sensor values once and print the result to the console. Run it as

    $ python bme280.py --help

to print the command-line help:

    usage: bme280.py [-h] [--bus BUS] [--addr ADDR] [--os OS]

    optional arguments:
      -h, --help   show this help message and exit
      --bus BUS    I2C Bus Number, set to 1 to access /dev/i2c-1.
      --addr ADDR  Sensor I2C address
      --os OS      Oversampling setting. 1=1x, 2=2x, 3=4x, 4=8x, 5=16x.
        
The output looks like this:

    $ python bme280.py --os 5
    temperature=20.9580170684
    pressure=1013.00577913
    humidity=43.6237501913

