function Decoder(data, port) {
  var lat = (data[0] | data[1]<<8 | data[2]<<16) / 10000;
  var lng = (data[3] | data[4]<<8 | data[5]<<16) / 10000;
  var alt = (data[6] | data[7]<<8);
  var dop = (data[8] | data[9]<<8) / 100;
  return {
    location: {
    latitude: lat,
    longitude: lng,
    altitude: alt,
    hdop: dop
    }
  };
}
