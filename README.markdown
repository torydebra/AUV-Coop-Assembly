# MASTER THESIS in development


eigen not found soluz
dovuto al fatto che eigen è dentro la cartella eigen3
root@Aspire-E15:/usr/include# ln -s /usr/include/eigen3/Eigen Eigen





cmat library installation

boost (already installed?)

kdl (already installed with ros)

parse_urdf (already installed ros)




to add a task:
CMAKE:
add_library
add target_linl_libraries sia per task cpp sia metti sto task cpp come link al main


#### robe
in cmat_defines.h there is a #define PI 3.14....    this is a problem because kdl library has a const int called PI.
solution is to change in cmat_defines.h : from #define PI 3.14....  to const int PI 3.14....


Libraries:
**tf** (to get transformation from ros) usata solo dal (rosInterface) che gestisce le cose ros.

**cmat** solo per computazioni pseudoinverse in icat eq e ican ineq, so nota solo alla classe controller che effettivamente fa ste operazioni. Indipendente da ros

**eigen** matrice di "passaggio" tra le due sopra.
Usata anche per riempire le jacobiane nelle classi task (più comoda di cmat).
Nota a TASKS e (main) ma non è un problema perchè è indipendente da ros, (installabile anche senza ros)
controllore non la usa perchè usa solo cmat

