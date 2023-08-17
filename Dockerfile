FROM python
ENV semantix_port=7500

COPY MovementPlanner.py /var
COPY AbstractVirtualCapability.py /var

EXPOSE 9999
CMD python /var/MovementPlanner.py ${semantix_port}