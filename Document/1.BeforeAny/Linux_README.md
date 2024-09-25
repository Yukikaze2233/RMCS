# Linux 简单命令与操作教学

## 特殊文件目录
`/`:单独的一个斜杠是表示根目录，在容器中，我们的路径就是根目录下的`workspace/RMCS`,表现为`/workspace/RMCS`

`~`:波浪号是当前用户的主目录，Linux为每个用户创建了一个主目录，其路径为`/home/username`，通常在shell中简单的表示为`~`,比如路径`\home\alray\Document`会被显示为`~\Document`

`.` 当前目录

`..` 上一级

`...`上上一级


## 命名补全
`linux`中的`bash`和`zsh`会有自动补全的功能，比如，当你输入`chmo`并且按下tab，就会补全成为`chmod`

接下来我们介绍关于zsh的补全使用

zsh的补全没有大小写之分,会自动换成正确的命令

在zsh中使用上下方向键可以切换上一条，下一条指令，当你输入了一部分，会跳到和你输入的内容匹配的上一条指令

## source 命令
`source`会在当前shell环境中逐行执行指定文件中的命令，而不会启动一个新的子shell。这意味着脚本中的变量、函数和其他shell特性会在```当前shell```会话中生效。

## shell 
shell 是一种用户界面，允许用户与操作系统进行交互。它通常以`命令行界面`的形式出现，但也可以是图形用户界面。对于没有接触过Linux的人而言,那就是你的终端，要注意在`vs code`中，你可以通过快捷键`ctrl+~`打开当前系统的`shell`

常见的`shell`有`zsh`和`bash`，我们在容器中使用的是zsh,当你在`vs code`中`reopen in container`之后，打开的就是容器中的shell,如果你在`vs code`中`ssh`了远程的其他电脑，那么会打开其他电脑的`shell`

## cd 命令
用于切换当前工作目录

## Shell快捷键
`ctrl+shift+c`：复制

`ctrl+shift+v`：粘贴

`ctrl+c`:中断当前shell中执行的程序 