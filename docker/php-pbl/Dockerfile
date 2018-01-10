FROM php:7.0-apache

COPY pbl/ /var/www/html/

RUN apt-get update && \
    apt-get install -y \
      python2.7 \
      graphviz \
      && \
    rm -rf /var/lib/apt/lists/*

RUN echo "log_errors = On" >> /usr/local/etc/php/php.ini && \
    echo "error_log = /dev/stderr" >> /usr/local/etc/php/php.ini && \
    echo -n 0 > /var/log/bdd-visit-count.txt && \
    echo -n 0 > /var/log/pbl-visit-count.txt && \
    touch /var/log/name-and-org.txt && \
    chown www-data:www-data \
      /var/log/bdd-visit-count.txt \
      /var/log/pbl-visit-count.txt \
      /var/log/name-and-org.txt \
      && \
    mkdir -p /var/www/html/python/images && \
    chown -R www-data:www-data /var/www/html
