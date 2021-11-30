<?php

$t1files = file_get_contents('t1');
$array1 = preg_split("/\r\n|\n|\r/", $t1files);

$t2files = file_get_contents('t2');
$array2 = preg_split("/\r\n|\n|\r/", $t2files);

// key = filename minus static prefix
$sizes1 = [];
$sizes2 = [];

// key = basename of file/s
$shortnames1 = [];
$shortnames2 = [];

$shahashes1 = [];   // key = filename minus static prefix, value = hash
$shahashes2 = [];  // key = filehash , value =  filename minus static prefix


foreach($array1 as $t1) {

    $fname1 = "../Teensy4/ChRt/src/".$t1;
    if (! is_file($fname1)) continue; 
    $sizes1[$t1] = filesize($fname1);
    //echo "fname1:$fname1\n";
    $f1c = file_get_contents($fname1);

    $base1 = basename($t1);
    if ( ! array_key_exists($base1,$shortnames1) ) $shortnames1[$base1] = [];
    $shortnames1[$base1][] = $t1;
    $shahashes1[$t1] = hash('ripemd160', $f1c); // key = filename
}

foreach($array2 as $t2) {

    $fname2 = "modules/ChibiOS/os/".$t2;
    if ( ! is_file($fname2)) continue; 
    $sizes2[$t2] = filesize($fname2);
    //echo "fname2:$fname2\n";
    $f2c = file_get_contents($fname2);

    $base2 = basename($t2);
    if ( ! array_key_exists($base2,$shortnames2) ) $shortnames2[$base2] = [];
    $shortnames2[$base2][] = $t2;
    $shahashes2[hash('ripemd160', $f2c)] = $t2; // key =  hash
}

# for every file in t1, go looking for its similar mate in t2.. and emit into if found

foreach($array1 as $t1) {
    $base1 = basename($t1);
    $t = $shortnames2[$base1]??[];
    if ( $t ) {
        //echo "BASE: srcpath:$t1 -> $base1 -> destpath:\n";
        $maxsim = 0;
        $best = [];
        foreach ( $t  as $k => $v) {
            $f1 = "../Teensy4/ChRt/src/".$t1;
            $f2 = "modules/ChibiOS/os/".$v;
            $f1c = file_get_contents($f1);
            $f2c = file_get_contents($f2);
            $perc = 0;
            $sim = similar_text($f1c, $f2c, $perc);
            
            if ( $perc > $maxsim) {
                $maxsim = $perc;
                $best = $f2;
                $bestf1 = $f1;
            }
        }
        //echo "$bestf1 -> $best \t\t similarity: $maxsim %\n";
        echo "diff $bestf1  $best  $maxsim %\n";
        
        //print_r($t);
    }
    $h = $shahashes1[$t1]??0 ;
    if ( $shahashes2[$h]??0 ) {
        $x = $shahashes2[$h];
        $f1 = "../Teensy4/ChRt/src/".$t1;
        $f2 = "modules/ChibiOS/os/".$x;
        //echo "HASH srcpath:$f1 -> $base1 -> destpath:$f2 \n";
        echo "HASH match:  $f1  $f2 \n";
    }

}