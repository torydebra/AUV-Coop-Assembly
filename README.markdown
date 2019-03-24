eigen not found soluz
dovuto al fatto che eigen è dentro la cartella eigen3
root@Aspire-E15:/usr/include# ln -s /usr/include/eigen3/Eigen Eigen





cmat library installation

boost (already installed?)




to add a task:
CMAKE:
add_library
add target_linl_libraries sia per task cpp sia metti sto task cpp come link al main



Libraries:
**tf** (to get transformation from ros) usata solo dal (rosInterface) che gestisce le cose ros.

**cmat** solo per computazioni pseudoinverse in icat eq e ican ineq, so nota solo alla classe controller che effettivamente fa ste operazioni. Indipendente da ros

**eigen** matrice di "passaggio" tra le due sopra.
Usata anche per riempire le jacobiane nelle classi task (più comoda di cmat).
Nota a TASKS e (main) ma non è un problema perchè è indipendente da ros, (installabile anche senza ros)
controllore non la usa perchè usa solo cmat

