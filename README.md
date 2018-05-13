# ArduinoGPSTracker

The goal of this project was to create a small GPS tracker powered by the smallest arduino possible aka arduino nano and the use
of the famous MQTT protocoll for integration with IBM Cloud.
NODE-RED was also used to create some workflows in order to integrate the data generated by the GPStracker with other systems but
this will not be covered here for now.

## Getting Started

These instructions will try to help you build the project on your own.I will try to be quick and simple

### Prerequisites

Hardware
```
Arduino nano(Any compatible board will do really)
SIM808 GPS/GSM Module
5V 2A DC power supply(SIM808 is really power hungry)
```
Software-Libraries
```
Arduino IDE
PubSubClient
arduinoJSON
TinyGsmClient
```

### Configuring Hardware

SIM808 TO ARDUINO
```
TX  -> PIN4
RX  -> PIN3
GND -> GND
VIN -> 5V
```
### Configuring Software

GSM Connection
```
To achieve a gsm connection we will be using TinyGSMClient library.
You will have to find your sim provider APN settings and plug them like this

const char apn[]  = "internet.vodafone.gr";
const char user[] = "";
const char pass[] = "";

```
##Work in progress.....
<!---  
End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc


--->
