#!/bin/bash -e

python /srv/jupyterhub/add-users.py /srv/jupyterhub/users.txt

jupyterhub
# jupyterhub >> jupyterhub.log 2>&1
