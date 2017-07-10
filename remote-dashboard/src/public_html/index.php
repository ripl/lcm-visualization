<?php
$BASE_HOST = $_SERVER['HTTP_HOST'];

function addAnalogIndicator( $id, $size, $x, $y, $min_angle, $max_angle, $min_value, $max_value ){
    global $BASE_HOST;
    ?>
    <img id="<?php echo $id ?>" class="indicator-<?php echo $size ?>"
         style="left:<?php echo $x ?>px; top:<?php echo $y ?>px; transform: rotate(<?php echo $min_angle ?>deg)"
         src="http://<?php echo $BASE_HOST; ?>/images/indicator_<?php echo $size ?>.png"
         data-min-angle="<?php echo $min_angle ?>" data-max-angle="<?php echo $max_angle ?>"
         data-min-value="<?php echo $min_value ?>" data-max-value="<?php echo $max_value ?>">
    <?php
}

function addImageIndicator( $id, $w, $h, $x, $y, $values, $default ){
    global $BASE_HOST;
    ?>
    <img id="<?php echo $id ?>" class="indicator-images"
         style="width:<?php echo $w ?>px; height:<?php echo $h ?>px; left:<?php echo $x ?>px; top:<?php echo $y ?>px"
         src="http://<?php echo $BASE_HOST; ?>/images/<?php echo $id.'_'.$default ?>.png"
         <?php
         foreach ($values as $v) {
             echo 'data-'.$v.'="http://'.$BASE_HOST.'/images/'.$id.'_'.$v.'.png" ';
         }
         ?>
    >
    <?php
}

function addTextIndicator( $id, $w, $x, $y, $color, $default ){
    ?>
    <div id="<?php echo $id ?>" class="indicator-text"
         style="width:<?php echo $w ?>px; left:<?php echo $x ?>px; top:<?php echo $y ?>px; color: <?php echo $color ?>"
    >
    <?php echo $default ?>
    </div>
    <?php
}
?>

<!doctype html>
<html lang="en">

<head>
    <meta charset="utf-8">
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <title>Clearpath Husky A200 Dashboard - by RIPL at TTIC</title>

    <script src="js/jquery.min.js"></script>

    <!-- Latest compiled and minified CSS -->
    <link rel="stylesheet" href="css/bootstrap.min.css">

    <!-- Latest compiled and minified JavaScript -->
    <script src="js/bootstrap.min.js"></script>

    <!-- custom style -->
    <link rel="stylesheet" href="css/style.css">

    <script type="text/javascript" src="http://maps.google.com/maps/api/js?sensor=false"></script>

    <style>
    .body-div{
        padding:80px 40px;
        width:1096px;
        height:720px;
        margin:0;
        background-image: url("http://<?php echo $BASE_HOST; ?>/images/background.png");
    }

    .indicator-images{
        position: absolute;
        z-index: 10;
        display: block;
        -moz-box-sizing: border-box;
        box-sizing: border-box;
    }

    .indicator-small{
        position: absolute;
        z-index: 10;
        width: 126px;
        height: 126px;
        display: block;
        -moz-box-sizing: border-box;
        box-sizing: border-box;
    }

    .indicator-large{
        position: absolute;
        z-index: 10;
        width: 240px;
        height: 240px;
        display: block;
        -moz-box-sizing: border-box;
        box-sizing: border-box;
    }

    .indicator-text{
        position: absolute;
        z-index: 10;
        display: block;
        -moz-box-sizing: border-box;
        box-sizing: border-box;
        font-family: "Helvetica Neue", Helvetica, Arial, sans-serif;
    	font-size: 24px;
    	font-style: bold;
    	font-variant: bold;
        text-align: center;
    }
    </style>
</head>

<body>

    <!-- portrait: 980x1185, landscape: 1096x720 -->
    <div class="body-div"></div>

    <canvas id='camera_frame_canvas' width="1096" height="360" style="position:absolute; top:0px; left:0px; z-index:-20"></canvas>
    <div id="map_canvas" style="width:310px; height:200px; position:absolute; display:block; left:393px; top:426px; z-index:-10"></div>

    <?php
    addAnalogIndicator('temperature', 'small', 100, 544, 0, 96, 20, 50);
    addAnalogIndicator('ram', 'small', 205, 507, 0, 72, 0, 1);
    addAnalogIndicator('cpu', 'large', 187, 428, 0, -240, 0, 1);
    addAnalogIndicator('linear_speed', 'large', 669, 428, 0, 240, 0, 4);
    addAnalogIndicator('battery', 'small', 870, 543, -50, -144, 0, 1);

    addImageIndicator('gearbox', 107, 26, 494, 613, array('P','N','D','R'), 'P');
    addImageIndicator('compass', 60, 60, 520, 360, array('-','W','NW','N','NE','E','SE','S','SW'), '-');

    addImageIndicator('lights', 71, 71, 387, 354, array('off','on'), 'off');
    addImageIndicator('autopilot', 71, 71, 448, 348, array('off','on'), 'off');
    addImageIndicator('faults', 71, 71, 583, 348, array('off','on'), 'off');
    addImageIndicator('e_stop', 71, 71, 642, 354, array('off','on'), 'off');

    addImageIndicator('roll', 269, 146, 7, 102, array(), 'static_white');
    addImageIndicator('pitch', 269, 168, 817, 89, array(), 'static_white');

    addTextIndicator('roll_txt', 170, 58, 89, 'white', '0&deg;' );
    addTextIndicator('pitch_txt', 170, 870, 89, 'white', '0&deg;' );
    ?>

    <script>
        <?php
        $maps_settings = file_get_contents('./maps.settings.json', true);
        ?>
        var map = null;
        var marker = null;

        function pinSymbol(color, rotation) {
            return {
                path: 'M 0,0 C -2,-20 -10,-22 -10,-30 A 10,10 0 1,1 10,-30 C 10,-22 2,-20 0,0 z',
                fillColor: color,
                fillOpacity: 1,
                strokeColor: '#000',
                strokeWeight: 1,
                rotation: rotation,
                scale: 1
            };
        }




        var RotateIcon = function(options){
            this.options = options || {};
            this.rImg = options.img || new Image();
            this.rImg.src = this.rImg.src || this.options.url || '';
            this.options.width = this.options.width || this.rImg.width || 20;
            this.options.height = this.options.height || this.rImg.height || 18;
            var canvas = document.createElement("canvas");
            canvas.width = this.options.width;
            canvas.height = this.options.height;
            this.context = canvas.getContext("2d");
            this.canvas = canvas;
        };
        RotateIcon.makeIcon = function(url) {
            return new RotateIcon({url: url, width: 20, height: 18});
        };
        RotateIcon.prototype.setRotation = function(options){
            var canvas = this.context;
            // ,
            //     angle = options.deg ? options.deg * Math.PI / 180: options.rad,
            //     centerX = this.options.width/2,
            //     centerY = this.options.height/2;

            // canvas.clearRect(0, 0, this.options.width, this.options.height);
            // canvas.save();
            // canvas.translate(-10, -10);
            // canvas.rotate(angle);
            // canvas.translate(-centerX, -centerY);
            console.log(this.rImg);
            canvas.drawImage(this.rImg, 0, 0, 20, 18);
            console.log(this.canvas.toDataURL());
            // canvas.restore();
            return this;
        };
        RotateIcon.prototype.getUrl = function(){
            return this.canvas.toDataURL('image/png');
        };



        function initMap() {
            map = new google.maps.Map(document.getElementById('map_canvas'), {
                zoom: 14,
                center: {lat: 41.7845294, lng: -87.5920523},
                mapTypeId: 'terrain',
                disableDefaultUI: true
            });

            icon_url = "http://<?php echo $BASE_HOST ?>/images/map_arrow.png";

            console.log(icon_url);

            marker_url = RotateIcon.makeIcon(icon_url).setRotation({deg: 0}).getUrl();
            console.log(marker_url);


            marker = new google.maps.Marker({
                position: {lat: 41.7845294, lng: -87.5920523},
                map: map,
                // icon: "http://<?php echo $BASE_HOST ?>/images/map_arrow.png"
                icon: marker_url
            });


            // map.setCenter(latlngbounds.getCenter());
        }

        initMap();

    </script>


    <script language="javascript" type="text/javascript">
        var ws_status = new WebSocket("ws://<?php echo $BASE_HOST ?>:8010/husky_status");
        var ws_camera = new WebSocket("ws://<?php echo $BASE_HOST ?>:8010/husky_camera");

        ws_status.onopen = function(){
            console.log("Connected to the 'status' app.");
        };

        ws_camera.onopen = function(){
            console.log("Connected to the 'camera' app.");
        };

        function updateAnalogIndicator( id, value ){
            value = parseFloat(value);
            min_angle = parseFloat($(id).data('min-angle'));
            max_angle = parseFloat($(id).data('max-angle'));
            min_value = parseFloat($(id).data('min-value'));
            max_value = parseFloat($(id).data('max-value'));
            value = Math.min( Math.max(value, min_value), max_value ) - min_value;
            rot_angle = min_angle + ( value / (max_value - min_value) ) * ( max_angle - min_angle );
            $(id).css('transform','rotate('+rot_angle+'deg)');
        }

        function updateImageIndicator( id, value, is_static ){
            if( is_static ){
                $(id).css('transform','rotate('+value+'deg)');
            }else{
                image_src = $(id).data(value.toLowerCase());
                $(id).attr('src', image_src);
            }
        }

        function updateTextIndicator( id, value, prefix, suffix ){
            $(id).html( prefix + Math.floor(value) + suffix );
        }

        function boolToOnOff( bool_val ){
            if( bool_val ){
                return 'on';
            }
            return 'off';
        }

        ws_status.onmessage = function(evt){
            data = JSON.parse(evt.data);
            // console.log(data);
            updateAnalogIndicator( "#temperature", data.temperature );
            updateAnalogIndicator( "#ram", data.ram );
            updateAnalogIndicator( "#cpu", data.cpu );
            updateAnalogIndicator( "#linear_speed", data.linear_speed );
            updateAnalogIndicator( "#battery", data.battery );

            updateImageIndicator( "#roll", data.roll, true );
            updateImageIndicator( "#pitch", data.pitch, true );
            updateTextIndicator( "#roll_txt", data.roll, "", "&deg;", false );
            updateTextIndicator( "#pitch_txt", data.pitch, "", "&deg;", false );

            updateImageIndicator( "#compass", data.compass, false );
            updateImageIndicator( "#gearbox", data.gearbox, false );

            updateImageIndicator( "#lights", boolToOnOff(data.lights_on), false );
            updateImageIndicator( "#autopilot", boolToOnOff(data.autopilot_on), false );
            updateImageIndicator( "#faults", boolToOnOff(data.faults_on), false );
            updateImageIndicator( "#e_stop", boolToOnOff(data.e_stop_on), false );
        };

        var canvas = document.getElementById('camera_frame_canvas');

        HTMLCanvasElement.prototype.renderImage = function(blob){
            var ctx = this.getContext('2d');
            var img = new Image();
            img.onload = function(){
                ctx.drawImage(img, 0, 0, 1096, 360);
            }
            var binaryData = [];
            binaryData.push(blob);
            img.src = URL.createObjectURL(new Blob(binaryData, {type: "image/jpeg"}));
        };

        ws_camera.onmessage = function(evt){
            canvas.renderImage(evt.data);
        };
    </script>

</body>
</html>
