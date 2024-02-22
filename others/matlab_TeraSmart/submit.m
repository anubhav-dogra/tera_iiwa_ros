%ip is TeraSmart PC address
%retlen is number of points you expect to receive
% boolfcts = ['START', 'STOP', 'CLEARBUFFER', 'RESET'];
% intfcts  = ['GETENDVALUE', 'GETSTARTVALUE', 'GETMODE', 'GETNUMBEROFPOINTS', 'GETSTATUS', 'GETFREQUENCY'];
% floatfcts= ['GETTIMEAXIS', 'GETTIMESTAMP', 'GETTIMEAXIS', 'GETLATESTPULSE','GETLATESTPULSE2', 'GETRAWPULSEFROMQUEUE', 'GETRAWPULSEFROMQUEUE2', 'GETPULSEFROMQUEUE', 'GETPULSEFROMQUEUE2'];

function R = submit(command,type_id,retlen)
    tcpObj = tcpclient('10.216.47.23', 8001);
    tcpObj.InputBufferSize=retlen*8;
    set(tcpObj, 'ByteOrder', 'little-endian');
    fopen(tcpObj);
    fwrite(tcpObj, command);
    R = fread(tcpObj, retlen, type_id);
    clear tcpObj
%     fclose(tcpObj);
%     delete(tcpObj);
end
