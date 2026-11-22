# RP2350 PWM phase shifting 

A simple PWM phase shifting example meant for power converters at high frequencies and high resolution, low cost solution for DYI/Industrial power Inverters/Converters. 

## Description

Pios state machines on RP2350 are used to achieve a high determinism level, along with high freequencies, this also achieves greater space for future code implementations or more complex algorithms as it releases tons of processing power from both cores. Future implementations expected: SOC estimator for Li-ion cells, SOC balancing control and GUI monitoring, GUI not uploaded. Also electrical diagrams not included, I'll try to upload them ASAP.

## Getting Started

### Dependencies

* VS Code pico add-on / native libraries required, can be compiled on any OS with Clang, Cmake, etc.

### Installing

* Clone the repository
* Compile the project
* Flash into MCU

## Authors

Jesus Gabriel Arroyo 
[@Email](gabriel.arroyop@pucp.edu.pe)

## Version History

* 0.1
    * Initial Release

## License

Not licenced yet

## Acknowledgments

Project part of academic research supported by Concytec-Prociencia by 
the Project “Manufactura de Sistemas de Recarga de 
Baterías de Vehículos eléctricos “Inteligentes”, con 
capacidad de Gestión de Microrredes Eléctricas, y basados 
sobre principios de Economía Circular” [Contract  Nº 
PE501094745-2025-PROCIENCIA] 

Please be kind and cite the articles this project is based on:

* [Springer_chapter](doi: 10.24084/eqj26-222)
* [Congress](doi: 10.1007/978-3-032-18982-0_10)