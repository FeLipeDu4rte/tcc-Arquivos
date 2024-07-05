<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Document</title>
    <link rel="stylesheet" href="style.css">
</head>
<body>
    <?php
        /*Quando fazer o banco de dados -- include('conexao.php');*/
        $nome = $_POST['nome'];
        $email = $_POST['email'];
        $numero = $_POST['numero'];
        $tipo_info = $_POST['contato'];
        echo "<h1>Dados do cadastro</h1>";
        echo "Nome: $nome<br>";
        echo "email: $email<br>";
        echo "numero: $numero<br>";
        echo "tipo: $tipo_info<br>";
        // INSER INTO cidade(nome, estado);
        // VALUES ('$nome', '$estado');
        /*$sql = "INSERT INTO cadastro (nome, email, numero)";
        $sql .= " VALUES('".$nome."','".$email."','".$numero."')";
        echo $sql;
        $result = mysqli_query($con,$sql);
        if($result){
            echo "<h2>Dados cadastrados com sucesso!!</h2>";

        }else{
            echo "<h2>Erro ao cadastrar!</h2>";
            echo mysqli_error($con);
        }
        */
    ?>
</body>
</html>