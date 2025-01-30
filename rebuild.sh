docker container stop crazysim_icuas_cont 
docker container rm crazysim_icuas_cont
docker build --ssh default --build-arg CACHE_DATE=$(date +%s) -t crazysim_icuas_img .
./first_run.sh
