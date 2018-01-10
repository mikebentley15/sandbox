FROM ubuntu:16.04

# install dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
      wget \
      bzip2 \
      sudo \
      && \
    rm -rf /var/lib/apt/lists/*

# add a separate non-root user with sudo permissions
# username: labuser
# password: labpass
RUN useradd --create-home --shell /bin/bash --gid sudo labuser && \
    echo 'labuser:labpass' | chpasswd

# make following commands run as this new labuser user
USER labuser
WORKDIR /home/labuser

# install anaconda
RUN wget https://repo.continuum.io/archive/Anaconda3-5.0.1-Linux-x86_64.sh && \
    bash Anaconda3-5.0.1-Linux-x86_64.sh -b && \
    rm Anaconda3-5.0.1-Linux-x86_64.sh

# add jupyter-lab
RUN echo "export PATH=\$PATH:\$HOME/anaconda3/bin" >> .bashrc
RUN ./anaconda3/bin/conda install -y jupyterlab

# configure for any IP to connect to it
RUN ./anaconda3/bin/jupyter lab --generate-config && \
    sed -i -e "s/#c.NotebookApp.ip = .*/c.NotebookApp.ip = '*'/g" \
        .jupyter/jupyter_notebook_config.py

# Copy the Dockerfile used to generate the image
COPY Dockerfile .

# Expose port 8000
EXPOSE 8000

# Run jupyter lab
CMD ["./anaconda3/bin/jupyter", "lab", "-y", "--no-browser", \
     "--port=8000", "--port-retries=0"]
