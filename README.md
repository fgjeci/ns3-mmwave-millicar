# mmWave ns-3 module #

This is an [ns-3](https://www.nsnam.org "ns-3 Website") forked from this [repository](https://github.com/signetlabdei/millicar "Millicar") module for the simulation of Vehicular & 5G cellular networks operating at mmWaves. 



## Installation
For a full installation starts from the [Millicar - O-RAN integration](https://github.com/fgjeci/millicar-oran-integration.git "Millicar - O-RAN integration") repository.

This repository contains only the ns-3 installation. 

Use these commands to download and build `ns3-mmwave-millicar`:
```
git clone https://github.com/fgjeci/ns3-mmwave-millicar.git
cd ns3-mmwave-millicar
./ns3 configure -d debug --disable-python --enable-examples && ./ns3 build
```

## Usage example
You can use the following command to run the `mmwave-simple-epc` example. 
```
./ns3 --run mmwave-simple-epc
```
Other examples are included in `src/mmwave/examples/`

## Publication

If you use this module, please cite:

F. Linsalata, E. Moro, F. Gjeci, M. Magarini, U. Spagnolini and A. Capone, "Addressing Control Challenges in Vehicular Networks Through O-RAN: A Novel Architecture and Simulation Framework," in IEEE Transactions on Vehicular Technology, vol. 73, no. 7, pp. 9344-9355, July 2024, doi: 10.1109/TVT.2024.3355202. [bibtex available here](https://ieeexplore.ieee.org/abstract/document/10401992?casa_token=TWpU68POP9EAAAAA:Xlz7_QlTv-bpkjZmL2DVw7xhyBgkPY7jsWhWC8zMmxI5bnqUn4ixuJM_yplhZiQiX8jP_bh-7A)

## Authors ##

The ns-3 mmWave module is the result of the development effort carried out by different people. The main contributors are: 
- Franci Gjeci, Politecnico di Milano University
- Eugenio Moro, Politecnico di Milano University
- Francesco Linsalata, Politecnico di Milano University

## License ##

This software is licensed under the terms of the GNU GPLv2, as like as ns-3. See the LICENSE file for more details.
