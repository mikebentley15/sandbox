FROM jupyterhub/jupyterhub

ENV DEBIAN_FRONTEND noninteractive

# Install basic dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
      npm \
      nodejs-legacy \
      && \
    pip install --upgrade pip && \
    rm -rf /var/lib/apt/lists/*

# Install Jupyter notebook
RUN pip install --upgrade jupyterhub && \
    pip install notebook

# Install dependencies for class notebooks
RUN apt-get update && \
    apt-get install -y \
      graphviz \
      && \
    pip install graphviz && \
    pip install ply && \
    pip install sly && \
    pip install ipywidgets && \
    pip install matplotlib && \
    rm -rf /var/lib/apt/lists/*

# Install Jupyter extensions
RUN pip install nbextensions && \
    pip install jupyter_contrib_nbextensions && \
    jupyter nbextension enable --py --sys-prefix widgetsnbextension && \
    jupyter nbextension install --py jupyter_contrib_nbextensions && \
    jupyter contrib nbextension install --sys-prefix --only-config
    #jupyter nbextension enable --py --sys-prefix \
    #  jupyter_contrib_nbextensions

#RUN echo "deb http://ftp.debian.org/debian jessie-backports main" \
#      > /etc/apt/sources.list.d/jessie-backports.list && \
#    apt-get update && \
#    apt-get -t jessie-backports install certbot && \
#    certbot certonly --standalone -d formal.cs.utah.edu && \
#    rm -rf /var/lib/apt/lists/*

#RUN git clone https://github.com/jupyterhub/dockerspawner.git && \
#    cd dockerspawner && \
#    pip install -r requirements.txt && \
#    pip install .

#RUN docker pull jupyterhub/singleuser

COPY jupyterhub_config.py ./
COPY startup.sh ./
COPY add-users.py ./
COPY skel /etc/skel/

EXPOSE 8000

CMD ["/bin/bash", "startup.sh"]
